#include "bsp/serial/serial_port.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>

namespace autoaim::bsp {

namespace {

speed_t to_speed(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return B115200;
    }
}

} // namespace

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string& dev, int baud) {
    std::lock_guard<std::mutex> lock(life_mutex_);
    closeUnlocked();   // 不能调 close()，会重复加锁死锁

    // O_NOCTTY  : 不要把本终端当作进程的控制终端
    // O_NONBLOCK: 先非阻塞打开，避免某些设备因 DCD 信号让 open 挂死
    const int fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return false;

    termios tty{};
    if (::tcgetattr(fd, &tty) != 0) {
        ::close(fd);
        return false;
    }

    // 原始模式：不做行处理、回显、特殊字符转换。
    // 二进制协议必须走原始模式，否则 0x0A / 0x0D 会被改写。
    ::cfmakeraw(&tty);

    tty.c_cflag |= (CLOCAL | CREAD);   // 忽略调制解调器控制线，启用接收
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;                // 8 数据位
    tty.c_cflag &= ~PARENB;            // 无校验
    tty.c_cflag &= ~CSTOPB;            // 1 停止位
    tty.c_cflag &= ~CRTSCTS;           // 无硬件流控

    // 组帧由 FrameParser 负责，所以让 read 尽快返回：
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    ::cfsetispeed(&tty, to_speed(baud));
    ::cfsetospeed(&tty, to_speed(baud));

    if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
        ::close(fd);
        return false;
    }

    ::tcflush(fd, TCIOFLUSH);   // 丢掉打开前残留在缓冲里的字节

    // 回到阻塞模式；超时由 select() 控制。
    const int flags = ::fcntl(fd, F_GETFL, 0);
    ::fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    fd_.store(fd);
    dev_ = dev;
    return true;
}

void SerialPort::close() {
    std::lock_guard<std::mutex> lock(life_mutex_);
    closeUnlocked();
}

void SerialPort::closeUnlocked() {
    // exchange 保证只有一个线程真正执行 close
    const int fd = fd_.exchange(-1);
    if (fd >= 0) {
        ::close(fd);
    }
    // 故意不清空 dev_：它是 std::string，clear() 会和读它的线程竞争，
    // 而且清空对排查问题没有任何好处。
}

ssize_t SerialPort::write_bytes(const uint8_t* data, size_t len) {
    const int fd = fd_.load();
    if (fd < 0 || data == nullptr || len == 0) return -1;

    size_t written = 0;
    while (written < len) {
        const ssize_t n = ::write(fd, data + written, len - written);
        if (n > 0) {
            written += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && errno == EINTR) continue;   // 被信号打断，重试
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            ::usleep(200);                        // 输出缓冲满，稍等再试
            continue;
        }
        return -1;
    }
    return static_cast<ssize_t>(written);
}

ssize_t SerialPort::write_bytes(const void* data, size_t len) {
    return write_bytes(static_cast<const uint8_t*>(data), len);
}

ssize_t SerialPort::read_bytes(uint8_t* buf, size_t len, int timeout_ms) {
    const int fd = fd_.load();
    if (fd < 0 || buf == nullptr || len == 0) return -1;

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);

    timeval tv{};
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    const int r = ::select(fd + 1, &rfds, nullptr, nullptr, &tv);
    if (r == 0) return 0;                                  // 超时
    if (r < 0)  return (errno == EINTR) ? 0 : -1;

    const ssize_t n = ::read(fd, buf, len);
    if (n < 0 && (errno == EINTR || errno == EAGAIN)) return 0;
    return n;
}

}  // namespace autoaim::bsp

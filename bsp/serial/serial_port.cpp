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

}

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string& dev, int baud) {
    std::lock_guard<std::mutex> lock(life_mutex_);
    closeUnlocked();

    const int fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);// 打开串口设备文件
    if (fd < 0) return false;

    termios tty{};// 获取串口属性
    if (::tcgetattr(fd, &tty) != 0) {// 获取串口属性失败
        ::close(fd);
        return false;
    }

    ::cfmakeraw(&tty);

    tty.c_cflag |= (CLOCAL | CREAD);// 设置本地连接和接收使能
    tty.c_cflag &= ~CSIZE;// 清除数据位掩码
    tty.c_cflag |= CS8;// 设置数据位为8位
    tty.c_cflag &= ~PARENB;// 禁用奇偶校验
    tty.c_cflag &= ~CSTOPB;// 禁用停止位
    tty.c_cflag &= ~CRTSCTS;// 禁用RTS/CTS流控制

    tty.c_cc[VMIN]  = 0;// 设置最小读取字节数为0
    tty.c_cc[VTIME] = 0;// 设置读取超时时间为0

    ::cfsetispeed(&tty, to_speed(baud));// 设置输入波特率
    ::cfsetospeed(&tty, to_speed(baud));// 设置输出波特率

    if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
        ::close(fd);
        return false;
    }

    ::tcflush(fd, TCIOFLUSH);

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
    const int fd = fd_.exchange(-1);
    if (fd >= 0) {
        ::close(fd);
    }
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
        if (n < 0 && errno == EINTR) continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            ::usleep(200);
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
    if (r == 0) return 0;
    if (r < 0)  return (errno == EINTR) ? 0 : -1;

    const ssize_t n = ::read(fd, buf, len);
    if (n < 0 && (errno == EINTR || errno == EAGAIN)) return 0;
    return n;
}

}

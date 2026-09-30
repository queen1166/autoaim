#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

namespace autoaim::bsp {

// 对 Linux termios 的薄封装。只做三件事：打开、写、读。
//
// 自瞄赛道走 USB CDC 虚拟串口：USB CDC 不使用固定的波特率、校验位或停止位，
// 串口设备的打开参数由 USB 驱动处理。这里仍然填 115200，是为了兼容
// 万一插到真 UART 上（导航赛道那种 USB-UART 转接）的情况。
//
// ── 线程安全 ──────────────────────────────────────────────────
// 这个类会被 txLoop 和 rxLoop【两个线程同时】使用（一个只写、一个只读）。
// 同一个 tty fd 上的并发 read/write 在系统调用层面是安全的，但 fd_ 本身
// 必须是原子的 —— 否则 close() 和 I/O 之间就是数据竞争。
//
// close() 与正在进行的 read/write 仍有内在竞争：关闭一个别的线程正在
// 阻塞的 fd，那个线程不保证被唤醒。所以调用方【必须】先让 I/O 线程退出
// 再析构（app/threads 的 Handles 析构函数会做这件事）。
class SerialPort {
public:
    // 构造：不打开任何东西，fd_ 保持 -1（is_open() 返回 false）。
    // 真正打开设备要显式调 open()。
    SerialPort() = default;

    // 析构：自动调 close()，所以持有 SerialPort 的对象一销毁，
    // fd 就还给了系统 —— 不写这行跑久了会耗尽 fd 表。
    ~SerialPort();

    // 禁止拷贝：两个对象共有一个 fd 会在析构时 double close，
    // 更糟的是可能把已经分配给别人（系统复用）的 fd 号给关掉。
    SerialPort(const SerialPort&)            = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // 打开串口设备并配置为原始模式（8N1、无流控、无回显）。
    //
    // dev  强烈建议用 /dev/serial/by-id/usb-...，
    //      因为重新插拔后 /dev/ttyACM* 的编号可能变化。
    // baud 波特率。USB CDC 虚拟串口下由驱动忽略，填 115200 只是为了
    //      兼容"万一插到真 UART 上"的情况。
    //
    // 返回 true 打开成功。失败时内部已清理，对象仍可再次 open()。
    // 注意：如果之前已打开，会先关掉旧的。
    bool open(const std::string& dev, int baud = 115200);

    // 关闭设备、释放 fd。可以重复调用（没打开时什么都不做）。
    // ⚠️ 调用方必须保证此刻没有别的线程正阻塞在 read_bytes() 里 ——
    //    关掉一个别人正在阻塞的 fd，那个线程不保证会被唤醒。
    void close();

    // 设备当前是否处于打开状态。线程安全（读的是原子 fd_）。
    bool is_open() const { return fd_.load() >= 0; }

    // 往串口写数据。返回实际写入的字节数；出错返回 -1。
    // 这是 18 字节指令帧的出口，由 txLoop 调用。
    //
    // 两个重载只是参数类型不同（uint8_t* / void*），
    // 方便调用方直接丢结构体指针进来，内部逐字节写出。
    ssize_t write_bytes(const uint8_t* data, size_t len);
    ssize_t write_bytes(const void* data, size_t len);

    // 阻塞直到读到至少 1 字节，或超时。由 rxLoop 调用。
    //
    // buf        接收缓冲
    // len        缓冲大小（要读多少）
    // timeout_ms 超时毫秒数
    //
    // 返回值：>0 实际字节数；0 超时（或被打断）；-1 出错。
    // ⚠️ 返回的字节数【不等于】一帧的长度 —— 一次 read 可能拿到半帧、
    //    整帧或多帧，必须交给 srm::FrameParser 做流式组帧。
    ssize_t read_bytes(uint8_t* buf, size_t len, int timeout_ms);

    // 当前设备的路径（就是 open() 传进来的那个字符串）。
    // open() 之后不再改动，所以读它是安全的。用来打日志。
    const std::string& device() const { return dev_; }

private:
    // 不加锁的关闭。open() 已经持有 life_mutex_，不能再调带锁的 close()。
    void closeUnlocked();

    // 被 txLoop 和 rxLoop 同时访问
    std::atomic<int> fd_{-1};
    // open() 时写一次，之后只读；close() 不清空（避免与读者竞争）
    std::string dev_;
    // 只保护 open/close，不碰热路径
    std::mutex life_mutex_;
};

}  // namespace autoaim::bsp

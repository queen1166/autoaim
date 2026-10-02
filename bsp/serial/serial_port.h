#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

namespace autoaim::bsp {

class SerialPort {
public:
    SerialPort() = default;

    ~SerialPort();

    SerialPort(const SerialPort&)            = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open(const std::string& dev, int baud = 115200);

    void close();

    bool is_open() const { return fd_.load() >= 0; }

    ssize_t write_bytes(const uint8_t* data, size_t len);
    ssize_t write_bytes(const void* data, size_t len);

    ssize_t read_bytes(uint8_t* buf, size_t len, int timeout_ms);

    const std::string& device() const { return dev_; }

private:
    void closeUnlocked();

    std::atomic<int> fd_{-1};
    std::string dev_;
    std::mutex life_mutex_;
};

}

#include "serial_comm.h"
#include <vector>
#include <string>
#include <cstring>
#include <boost/optional.hpp>
#include <exception>

#if defined(_WIN32) || defined(_WIN64)
    #include <windows.h>
#else
    #include <boost/filesystem.hpp>
#endif

static constexpr int32_t err_buf_size = 1024;
static constexpr int32_t read_buf_size = 1024;

struct SerialPort {
    boost::asio::io_service io_service;
    boost::optional<boost::asio::serial_port> serial;
    char error_buffer[err_buf_size] = {0};
    uint8_t read_buffer[read_buf_size] = {0};
};

struct ScanResult {
    std::vector<std::string> available_ports;
};

const char* get_last_serial_port_error(uint64_t handle) {
    if (handle == 0) return "";
    SerialPort* serialPort = reinterpret_cast<SerialPort*>(handle);
    return serialPort->error_buffer;
}

uint64_t open_serial_port(const char* port, uint32_t baud_rate) {
    try {
        SerialPort* handle = new SerialPort{};
        return reinterpret_cast<uint64_t>(handle);
    } catch (std::exception& e) {
        // Memory failure- obviously😂
    }
    return 0; // Error
}

bool connect_serial_port(uint64_t serial_port_handle, const char* port, uint32_t baud_rate) {
    if (serial_port_handle == 0) {
        return false; // Error
    }
    SerialPort* serialPort = reinterpret_cast<SerialPort*>(handle);
    serialPort->error_buffer[0] = '\0'; // Reset error buffer
    try {
        handle->serial.emplace(handle->io_service);
        handle->serial.open(port);
        handle->serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    } catch (std::exception& e) {
        strncpy(handle->error_buffer, e.what(), err_buf_size - 1)[err_buf_size - 1] = '\0';
        return false; // Error
    }
    return true;
}

void close_serial_port(uint64_t handle) {
    if (handle == 0)
        return;
    SerialPort* serialPort = reinterpret_cast<SerialPort*>(handle);
    if (serialPort->serial.has_value()) {
        serialPort->serial.close();
        serialPort->serial = boost::none;
    }
    delete serialPort;
};

bool write_serial(uint64_t handle, const uint8_t* data) {
    if (handle == 0) {
        return false; // Error
    }
    SerialPort* serialPort = reinterpret_cast<SerialPort*>(handle);
    serialPort->error_buffer[0] = '\0'; // Reset error buffer
    try {
        boost::asio::write(serialPort->serial, boost::asio::buffer(data, strlen(data)));
        return true;
    } catch (std::exception& e) {
        strncpy(handle->error_buffer, e.what(), err_buf_size - 1)[err_buf_size - 1] = '\0';
    }
    return false; // Error
}

const uint8_t* read_serial(uint64_t handle, bool* was_error) {
    *was_error = false;
    if (handle == 0) {
        *was_error = true; // Error
        return reinterpret_cast<const uint8_t*>("");
    }
    SerialPort* serialPort = reinterpret_cast<SerialPort*>(handle);
    serialPort->error_buffer[0] = '\0'; // Reset error buffer
    try {
        boost::asio::read(serialPort->serial, boost::asio::buffer(serialPort->read_buffer, read_buf_size));
        return serialPort->read_buffer;
    } catch (std::exception& e) {
        *was_error = true; // Error
        strncpy(handle->error_buffer, e.what(), err_buf_size - 1)[err_buf_size - 1] = '\0';
    }
    return reinterpret_cast<const uint8_t*>("");
}

#if defined(_WIN32) || defined(_WIN64)
void enumerateWindowsPorts() {
    // Windows-specific port enumeration
    char portName[32];
    for (int i = 1; i <= 256; ++i) {
        sprintf_s(portName, "\\\\.\\COM%d", i);
        HANDLE hPort = CreateFileA(portName, GENERIC_READ | GENERIC_WRITE,
                                   0, NULL, OPEN_EXISTING, 0, NULL);
        if (hPort != INVALID_HANDLE_VALUE) {
            available_ports.push_back(portName);
            CloseHandle(hPort);
        }
    }
}
#else
void enumerateLinuxPorts() {
    // Linux-specific port enumeration using Boost
    using namespace boost::filesystem;
    path dev("/dev");
    if (exists(dev) && is_directory(dev)) {
        for (auto& p : directory_iterator(dev)) {
            if (is_character_file(p.status())) {
                std::string fileName = p.path().filename().string();
                if (fileName.find("ttyUSB") != std::string::npos || fileName.find("ttyS") != std::string::npos) {
                    available_ports.push_back(p.path().string());
                }
            }
        }
    }
}
#endif

uint64_t list_serial_ports() {
    ScanResult* handle = new ScanResult{};
    #if defined(_WIN32) || defined(_WIN64)
        enumerateWindowsPorts();
    #else
        enumerateLinuxPorts();
    #endif

    *count = available_ports.size();
    const char** ports = new const char*[*count];
    for (int i = 0; i < *count; ++i) {
        ports[i] = available_ports[i].c_str();
    }

    return ports;
}

int32_t get_amount_available_ports(uint64_t handle);

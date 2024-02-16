#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include <boost/asio.hpp>
#include <vector>
#include <string>
#include <cstdint>

extern "C" {
    // Returns serial port handle. Returns 0 upon failure
    uint64_t open_serial_port();

    // Accepts the handle returned by open_serial_port.
    // Returns true if success.
    // If returns false- call get_last_serial_port_error with the handle.
    bool connect_serial_port(uint64_t serial_port_handle, const char* port, uint32_t baud_rate);

    // Accepts serial port handle that was created by open_serial_port
    // Returns either the error or an empty string if the handle
    // is invalid or if there was no error.
    const char* get_last_serial_port_error(uint64_t handle);

    // Accepts serial port handle that was created by open_serial_port
    // Disconnects and closes the port
    // Allowed to close a 0 handle, allowed to close a port that wasn't connected.
    void close_serial_port(uint64_t handle);

    // Accepts serial port handle that was created by open_serial_port
    // Accepts byte array to write
    // Return true if success.
    // If returns false- call get_last_serial_port_error with the handle.
    bool write_serial(uint64_t handle, const uint8_t* data);

    // Accepts serial port handle that was created by open_serial_port
    // Accepts out parameter was_error.
    // If was_error was set to false false- call get_last_serial_port_error with the handle.
    const uint8_t* read_serial(uint64_t handle, bool* was_error);


    // Returns serial scan result handle, or 0 if error.
    uint64_t open_serial_port_scan_object();

    // Accepts handle that was created by open_serial_port_scan_object
    // Returns either the error or an empty string if the handle
    // is invalid or if there was no error.
    const char* get_last_list_serial_ports_error(uint64_t handle);

    // Accepts handle returned by open_serial_port_scan_object
    // Return true if success.
    // If returns false- call get_last_list_serial_ports_error with the handle.
    // If returns true- call get_amount_available_ports to prepare for manual
    // iteration.
    bool list_serial_ports(uint64_t handle);

    // Accepts handle returned by open_serial_port_scan_object
    // Returns the number of serial ports available after calling list_serial_ports.
    int32_t get_amount_available_ports(uint64_t handle);

    // Accepts the handle returned by open_serial_port_scan_object
    // Cleans up resources allocated during the scan operation.
    void close_serial_port_scan_object(uint64_t handle);
}

#endif // SERIAL_COMM_H

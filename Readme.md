# WIP
Project outline and system design is completed, implementation requires additional care to the "list📃 serial ports" functionality which isn't complete yet.

# SerialCommunicationLib

SerialCommunicationLib is a comprehensive and easy-to-use C++ library designed to facilitate serial port communication across various platforms. Leveraging the power of Boost.Asio for asynchronous I/O, this library offers developers a straightforward approach to interfacing with hardware devices. Whether you're building applications that require communication with serial devices or developing embedded systems, SerialCommunicationLib provides the essential tools for opening, connecting, reading from, and writing to serial ports.

## Features

- Cross-platform support for Windows and Linux.
- Simplified API for opening and closing serial ports.
- Efficient reading and writing with serial devices.
- Error handling capabilities to ensure robust communication.
- Serial port scanning to identify available devices.

## Getting Started

To get started with SerialCommunicationLib, clone the repository to your local machine:

git clone https://github.com/AviadCohen24/SerialCommunicationLib.git

### Prerequisites

Ensure you have Boost.Asio installed on your system, as this library relies on it for asynchronous I/O operations.

### Usage Example

Here's a quick example of how to open a serial port, write to it, and then read from it:

```cpp
#include "serial_comm.h"

int main() {
    // Open serial port
    uint64_t port_handle = open_serial_port("COM3", 9600);
    if (port_handle == 0) {
        std::cerr << "Failed to open serial port" << std::endl;
        return -1;
    }

    // Write to serial port
    const char* data = "Hello, Serial Port!";
    if (!write_serial(port_handle, reinterpret_cast<const uint8_t*>(data))) {
        std::cerr << "Failed to write to serial port" << std::endl;
        return -1;
    }

    // Read from serial port
    bool was_error = false;
    const uint8_t* read_data = read_serial(port_handle, &was_error);
    if (was_error) {
        std::cerr << "Failed to read from serial port" << std::endl;
        return -1;
    }

    std::cout << "Received: " << read_data << std::endl;

    // Close the serial port
    close_serial_port(port_handle);

    return 0;
}
```

## Contributing

Contributions are what make the open-source community such an amazing place to learn, inspire, and create. Any contributions you make are greatly appreciated.

## Acknowledgments

	•	Boost.Asio for providing the asynchronous I/O capabilities.
	•	Aviad Cohen for creating and maintaining this library.

We hope SerialCommunicationLib empowers you to build incredible projects with ease and efficiency. Happy coding!

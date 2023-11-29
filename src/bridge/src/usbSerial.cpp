#include "usbSerial.h"
#include <iostream>
#include <stdexcept>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <termios.h>

USBSerial::USBSerial() : usbFileDescriptor(-1) {
    // Constructor body
    // Initialize any required members, if necessary
}

USBSerial::~USBSerial() {
    closeUSBPort(); // Ensure the port is closed on destruction
}

void USBSerial::openUSBPort(const std::string& devicePath, speed_t baudRate) {
    if (usbFileDescriptor >= 0) {
        closeUSBPort(); // Close existing descriptor if open
    }

    usbFileDescriptor = open(devicePath.c_str(), O_RDWR | O_NOCTTY);
    if (usbFileDescriptor < 0) {
        throw std::runtime_error("Failed to open USB port");
    }

    memset(&ioStruct, 0, sizeof(ioStruct));
    ioStruct.c_iflag = 0;
    ioStruct.c_oflag = 0;
    ioStruct.c_cflag = CS8 | CREAD | CLOCAL;
    ioStruct.c_lflag = 0;
    ioStruct.c_cc[VMIN] = 1;
    ioStruct.c_cc[VTIME] = 5;

    cfsetospeed(&ioStruct, baudRate);
    cfsetispeed(&ioStruct, baudRate);

    if (tcsetattr(usbFileDescriptor, TCSANOW, &ioStruct) != 0) {
        throw std::runtime_error("Failed to set terminal attributes");
    }
}

void USBSerial::sendData(const std::string& data) {
    if (write(usbFileDescriptor, data.c_str(), data.size()) < 0) {
        throw std::runtime_error("Failed to send data");
    }
}

std::string USBSerial::readData() {
    memset(serialDataIn, '\0', USBSerial::BUFFER_SIZE);
    int readBytes = read(usbFileDescriptor, serialDataIn, USBSerial::BUFFER_SIZE - 1);
    if (readBytes < 0) {
        throw std::runtime_error("Failed to read data");
    }
    return std::string(serialDataIn, readBytes);
}

void USBSerial::closeUSBPort() {
    if (usbFileDescriptor >= 0) {
        close(usbFileDescriptor);
        usbFileDescriptor = -1;
    }
}


#ifndef USBSERIAL_H
#define USBSERIAL_H

#include <string>
#include <termios.h>

class USBSerial {
public:
    USBSerial();
    virtual ~USBSerial();

    void openUSBPort(const std::string& devicePath, speed_t baudRate);
    void sendData(const std::string& data);
    std::string readData();
    void closeUSBPort();

private:
    struct termios ioStruct;
    int usbFileDescriptor;
    static const int BUFFER_SIZE = 1024;
    char serialDataIn[BUFFER_SIZE];
};

#endif // USBSERIAL_H


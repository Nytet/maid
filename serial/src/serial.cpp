#include "serial/serial.h"
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <errno.h>
#include <string.h>

namespace serial {

class Serial::SerialImpl {
public:
  SerialImpl() : fd_(-1), is_open_(false) {}

  void open(const std::string &port, uint32_t baudrate) {
    if (is_open_) {
      throw std::runtime_error("Serial port already open");
    }

    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error("Failed to open port: " + port);
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd_, &tty) != 0) {
      throw std::runtime_error("Failed to get port attributes");
    }

    // Set baud rate
    speed_t baud = B115200;  // Default to 115200
    switch (baudrate) {
      case 9600: baud = B9600; break;
      case 19200: baud = B19200; break;
      case 38400: baud = B38400; break;
      case 57600: baud = B57600; break;
      case 115200: baud = B115200; break;
      default: baud = B115200; break;
    }
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    // Set other port settings
    tty.c_cflag |= (CLOCAL | CREAD);    // Ignore modem controls
    tty.c_cflag &= ~PARENB;             // No parity
    tty.c_cflag &= ~CSTOPB;             // 1 stop bit
    tty.c_cflag &= ~CSIZE;              // Clear size bits
    tty.c_cflag |= CS8;                 // 8 bits
    tty.c_cflag &= ~CRTSCTS;            // No hardware flow control

    // Raw input
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // Raw output
    tty.c_oflag &= ~OPOST;

    // Read settings
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      throw std::runtime_error("Failed to set port attributes");
    }

    is_open_ = true;
  }

  void close() {
    if (is_open_) {
      ::close(fd_);
      fd_ = -1;
      is_open_ = false;
    }
  }

  bool isOpen() const {
    return is_open_;
  }

  size_t read(uint8_t *buf, size_t size) {
    if (!is_open_) {
      throw std::runtime_error("Port not open");
    }
    return ::read(fd_, buf, size);
  }

  size_t write(const uint8_t *data, size_t size) {
    if (!is_open_) {
      throw std::runtime_error("Port not open");
    }
    return ::write(fd_, data, size);
  }

  size_t available() {
    if (!is_open_) {
      throw std::runtime_error("Port not open");
    }
    int bytes_available;
    if (ioctl(fd_, FIONREAD, &bytes_available) < 0) {
      throw std::runtime_error("Failed to get available bytes");
    }
    return bytes_available;
  }

private:
  int fd_;
  bool is_open_;
};

// Serial class implementation
Serial::Serial() : pimpl_(new SerialImpl()) {}

Serial::Serial(const std::string &port, uint32_t baudrate) : pimpl_(new SerialImpl()) {
  pimpl_->open(port, baudrate);
}

Serial::~Serial() {
  close();
}

void Serial::open() {
  pimpl_->open(getPort(), getBaudrate());
}

void Serial::close() {
  pimpl_->close();
}

bool Serial::isOpen() const {
  return pimpl_->isOpen();
}

size_t Serial::read(uint8_t *buf, size_t size) {
  return pimpl_->read(buf, size);
}

size_t Serial::write(const uint8_t *data, size_t size) {
  return pimpl_->write(data, size);
}

size_t Serial::available() {
  return pimpl_->available();
}

// Placeholder implementations for other methods
void Serial::setPort(const std::string &port) {}
std::string Serial::getPort() const { return ""; }
void Serial::setTimeout(uint32_t timeout) {}
uint32_t Serial::getTimeout() const { return 0; }
void Serial::setBaudrate(uint32_t baudrate) {}
uint32_t Serial::getBaudrate() const { return 0; }
void Serial::setBytesize(uint8_t bytesize) {}
uint8_t Serial::getBytesize() const { return 0; }
void Serial::setParity(uint8_t parity) {}
uint8_t Serial::getParity() const { return 0; }
void Serial::setStopbits(uint8_t stopbits) {}
uint8_t Serial::getStopbits() const { return 0; }
void Serial::setFlowcontrol(uint8_t flowcontrol) {}
uint8_t Serial::getFlowcontrol() const { return 0; }
void Serial::setDTR(bool dtr) {}
void Serial::setRTS(bool rts) {}
bool Serial::getDTR() const { return false; }
bool Serial::getRTS() const { return false; }
std::vector<std::string> Serial::list_ports() { return std::vector<std::string>(); }

} // namespace serial 
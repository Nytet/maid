#ifndef SERIAL_H
#define SERIAL_H

#include <string>
#include <vector>
#include <cstring>
#include <sstream>
#include <exception>
#include <stdexcept>
#include <memory>

namespace serial {

class Serial {
public:
  Serial();
  Serial(const std::string &port, uint32_t baudrate);
  virtual ~Serial();

  void open();
  void close();
  bool isOpen() const;

  size_t available();
  bool waitReadable();
  void waitByteTimes(size_t count);

  size_t read(uint8_t *buf, size_t size);
  size_t write(const uint8_t *data, size_t size);

  void flush();
  void flushInput();
  void flushOutput();

  void setPort(const std::string &port);
  std::string getPort() const;

  void setTimeout(uint32_t timeout);
  uint32_t getTimeout() const;

  void setBaudrate(uint32_t baudrate);
  uint32_t getBaudrate() const;

  void setBytesize(uint8_t bytesize);
  uint8_t getBytesize() const;

  void setParity(uint8_t parity);
  uint8_t getParity() const;

  void setStopbits(uint8_t stopbits);
  uint8_t getStopbits() const;

  void setFlowcontrol(uint8_t flowcontrol);
  uint8_t getFlowcontrol() const;

  void setDTR(bool dtr);
  void setRTS(bool rts);
  bool getDTR() const;
  bool getRTS() const;

  static std::vector<std::string> list_ports();

private:
  class SerialImpl;
  std::unique_ptr<SerialImpl> pimpl_;
};

} // namespace serial

#endif // SERIAL_H 
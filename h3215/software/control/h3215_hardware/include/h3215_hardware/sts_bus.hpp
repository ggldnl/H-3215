// The servo bus over a serial port: the only implementation of the protocol, used by the
// control loop (StsSystem) and by servo_tool.
//
// Every buffer is allocated by open(): sync_read() and sync_write(), called by the control
// loop, never allocate.

#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace h3215_hardware
{

class StsBus
{
public:
  StsBus();
  ~StsBus();
  StsBus(const StsBus &) = delete;
  StsBus & operator=(const StsBus &) = delete;

  // Opens a serial port (or a pseudo-terminal, in the tests). Returns an error, empty on success
  std::string open(const std::string & port, int baudrate);
  void close();
  bool is_open() const { return fd_ >= 0; }
  int baudrate() const { return baudrate_; }

  // How long to wait for each reply
  void set_timeout(std::chrono::microseconds timeout) { timeout_ = timeout; }
  std::chrono::microseconds timeout() const { return timeout_; }

  bool ping(uint8_t id);
  // IDs of every servo that answers, waiting a short time for each: most IDs are absent
  std::vector<uint8_t> scan();
  // A write that the servo confirms
  bool write(uint8_t id, uint8_t address, const uint8_t * data, size_t size);
  bool read(uint8_t id, uint8_t address, uint8_t size, uint8_t * out);

  // The same registers from several servos with one request. Servo k's bytes go to
  // out[k * size], its error flags to errors[k], and received[k] tells whether it answered.
  // Returns how many answered.
  size_t sync_read(
    const std::vector<uint8_t> & ids, uint8_t address, uint8_t size, uint8_t * out,
    uint8_t * errors, bool * received);

  // data holds size bytes per servo, in the order of ids. Servos don't reply
  bool sync_write(
    const std::vector<uint8_t> & ids, uint8_t address, uint8_t size, const uint8_t * data);

private:
  bool send(uint8_t id, uint8_t instruction, const uint8_t * params, size_t size);
  // One status packet: id, error flags, and its params in reply_params_
  bool reply(uint8_t & id, uint8_t & error, size_t & size);
  bool read_exact(uint8_t * out, size_t size);

  int fd_ = -1;
  int baudrate_ = 0;
  std::chrono::microseconds timeout_{5000};
  std::vector<uint8_t> packet_;
  std::vector<uint8_t> params_;
  std::vector<uint8_t> reply_params_;
};

}  // namespace h3215_hardware

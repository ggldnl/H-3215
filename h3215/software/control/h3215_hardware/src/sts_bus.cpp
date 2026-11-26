#include "h3215_hardware/sts_bus.hpp"

#include <fcntl.h>
#include <linux/serial.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

#include "h3215_hardware/sts_protocol.hpp"

namespace h3215_hardware
{

namespace
{

speed_t to_speed(int baudrate)
{
  switch (baudrate) {
    case 115200: return B115200;
    case 500000: return B500000;
    case 1000000: return B1000000;
    default: return B0;
  }
}

}  // namespace

StsBus::StsBus()
{
  packet_.reserve(sts::kMaxPacket);
  params_.reserve(sts::kMaxPacket);
  reply_params_.resize(256);
}

StsBus::~StsBus() { close(); }

std::string StsBus::open(const std::string & port, int baudrate)
{
  close();
  const speed_t speed = to_speed(baudrate);
  if (speed == B0) {
    return "unsupported baud rate " + std::to_string(baudrate);
  }
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return "cannot open " + port + ": " + std::strerror(errno);
  }
  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    const std::string error = std::strerror(errno);
    close();
    return "cannot configure " + port + ": " + error;
  }
  cfmakeraw(&tty);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  tcsetattr(fd_, TCSANOW, &tty);

  // USB serial adapters (FTDI) otherwise hold received bytes for up to 16 ms. Not every
  // driver supports it, and CDC ACM adapters don't need it
  serial_struct serial{};
  if (ioctl(fd_, TIOCGSERIAL, &serial) == 0) {
    serial.flags |= ASYNC_LOW_LATENCY;
    ioctl(fd_, TIOCSSERIAL, &serial);
  }
  tcflush(fd_, TCIOFLUSH);
  baudrate_ = baudrate;
  return "";
}

void StsBus::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool StsBus::ping(uint8_t id)
{
  uint8_t rid, error;
  size_t size;
  return send(id, sts::kPing, nullptr, 0) && reply(rid, error, size) && rid == id;
}

std::vector<uint8_t> StsBus::scan()
{
  const auto timeout = timeout_;
  timeout_ = std::chrono::microseconds(5000);
  std::vector<uint8_t> ids;
  for (int id = 0; id < sts::kBroadcastId; ++id) {
    if (ping(static_cast<uint8_t>(id))) {
      ids.push_back(static_cast<uint8_t>(id));
    }
  }
  timeout_ = timeout;
  return ids;
}

bool StsBus::write(uint8_t id, uint8_t address, const uint8_t * data, size_t size)
{
  params_.clear();
  params_.push_back(address);
  params_.insert(params_.end(), data, data + size);
  uint8_t rid, error;
  size_t rsize;
  // Servos confirm every write (factory response level)
  return send(id, sts::kWrite, params_.data(), params_.size()) && reply(rid, error, rsize) &&
         rid == id;
}

bool StsBus::read(uint8_t id, uint8_t address, uint8_t size, uint8_t * out)
{
  const uint8_t params[] = {address, size};
  uint8_t rid, error;
  size_t rsize;
  if (!send(id, sts::kRead, params, 2) || !reply(rid, error, rsize) || rid != id ||
    rsize != size)
  {
    return false;
  }
  std::memcpy(out, reply_params_.data(), size);
  return true;
}

size_t StsBus::sync_read(
  const std::vector<uint8_t> & ids, uint8_t address, uint8_t size, uint8_t * out,
  uint8_t * errors, bool * received)
{
  const size_t n = ids.size();
  for (size_t k = 0; k < n; ++k) {
    received[k] = false;
  }
  params_.clear();
  params_.push_back(address);
  params_.push_back(size);
  params_.insert(params_.end(), ids.begin(), ids.end());
  if (!send(sts::kBroadcastId, sts::kSyncRead, params_.data(), params_.size())) {
    return 0;
  }

  // Servos answer in the order of ids. A missing one costs one timeout, then the rest still
  // arrive: each reply carries its id
  size_t count = 0;
  for (size_t attempt = 0; attempt < n && count < n; ++attempt) {
    uint8_t rid, error;
    size_t rsize;
    if (!reply(rid, error, rsize)) {
      continue;
    }
    for (size_t k = 0; k < n; ++k) {
      if (ids[k] == rid && !received[k] && rsize == size) {
        std::memcpy(out + k * size, reply_params_.data(), size);
        errors[k] = error;
        received[k] = true;
        ++count;
        break;
      }
    }
  }
  return count;
}

bool StsBus::sync_write(
  const std::vector<uint8_t> & ids, uint8_t address, uint8_t size, const uint8_t * data)
{
  params_.clear();
  params_.push_back(address);
  params_.push_back(size);
  for (size_t k = 0; k < ids.size(); ++k) {
    params_.push_back(ids[k]);
    params_.insert(params_.end(), data + k * size, data + (k + 1) * size);
  }
  return send(sts::kBroadcastId, sts::kSyncWrite, params_.data(), params_.size());
}

bool StsBus::send(uint8_t id, uint8_t instruction, const uint8_t * params, size_t size)
{
  if (fd_ < 0) {
    return false;
  }
  tcflush(fd_, TCIFLUSH);  // drop leftovers of an earlier failed exchange
  sts::make_packet(packet_, id, instruction, params, size);
  size_t sent = 0;
  while (sent < packet_.size()) {
    const ssize_t n = ::write(fd_, packet_.data() + sent, packet_.size() - sent);
    if (n < 0 && errno != EAGAIN && errno != EINTR) {
      return false;
    }
    if (n > 0) {
      sent += static_cast<size_t>(n);
    }
  }
  return true;
}

bool StsBus::reply(uint8_t & id, uint8_t & error, size_t & size)
{
  uint8_t head[4];  // FF FF id length
  if (!read_exact(head, 4) || head[0] != 0xFF || head[1] != 0xFF || head[3] < 2) {
    return false;
  }
  uint8_t rest[256];  // error flags, params, checksum
  if (!read_exact(rest, head[3])) {
    return false;
  }
  unsigned sum = head[2] + head[3];
  for (int i = 0; i < head[3] - 1; ++i) {
    sum += rest[i];
  }
  if (static_cast<uint8_t>(~sum & 0xFF) != rest[head[3] - 1]) {
    return false;
  }
  id = head[2];
  error = rest[0];
  size = head[3] - 2;
  std::memcpy(reply_params_.data(), rest + 1, size);
  return true;
}

bool StsBus::read_exact(uint8_t * out, size_t size)
{
  using clock = std::chrono::steady_clock;
  const auto deadline = clock::now() + timeout_;
  size_t got = 0;
  while (got < size) {
    const ssize_t n = ::read(fd_, out + got, size - got);
    if (n > 0) {
      got += static_cast<size_t>(n);
      continue;
    }
    if (n < 0 && errno != EAGAIN && errno != EINTR) {
      return false;
    }
    const auto left = std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - clock::now());
    if (left.count() <= 0) {
      return false;
    }
    pollfd pfd{fd_, POLLIN, 0};
    const timespec wait{
      static_cast<time_t>(left.count() / 1000000000), static_cast<long>(left.count() % 1000000000)};
    ppoll(&pfd, 1, &wait, nullptr);
  }
  return true;
}

}  // namespace h3215_hardware

#include "h3215_hardware/sts_protocol.hpp"

namespace h3215_hardware::sts
{

uint8_t checksum(const uint8_t * bytes, size_t size)
{
  unsigned sum = 0;
  for (size_t i = 0; i < size; ++i) {
    sum += bytes[i];
  }
  return static_cast<uint8_t>(~sum & 0xFF);
}

void make_packet(
  std::vector<uint8_t> & out, uint8_t id, uint8_t instruction, const uint8_t * params,
  size_t size)
{
  out.clear();
  out.push_back(0xFF);
  out.push_back(0xFF);
  out.push_back(id);
  out.push_back(static_cast<uint8_t>(size + 2));
  out.push_back(instruction);
  out.insert(out.end(), params, params + size);
  out.push_back(checksum(out.data() + 2, out.size() - 2));
}

int decode_signed(uint16_t value, int sign_bit)
{
  const int magnitude = value & ((1 << sign_bit) - 1);
  return (value >> sign_bit) & 1 ? -magnitude : magnitude;
}

State decode_state(const uint8_t * d)
{
  auto u16 = [d](int i) { return static_cast<uint16_t>(d[i] | (d[i + 1] << 8)); };
  return State{
    u16(0),
    decode_signed(u16(2), 15),
    decode_signed(u16(4), 10),
    d[6] / 10.0,
    d[7],
  };
}

}  // namespace h3215_hardware::sts

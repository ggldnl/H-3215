// Feetech STS servo protocol (STS3215).
//
// Request: FF FF <id> <length> <instruction> <params...> <checksum>
// Reply:   FF FF <id> <length> <error flags> <params...> <checksum>
// length = number of params + 2, checksum = ~(sum of the bytes after FF FF) & 0xFF.
// Multi-byte values are little-endian.

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace h3215_hardware::sts
{

constexpr uint8_t kPing = 0x01;
constexpr uint8_t kRead = 0x02;
constexpr uint8_t kWrite = 0x03;
constexpr uint8_t kSyncRead = 0x82;
constexpr uint8_t kSyncWrite = 0x83;
constexpr uint8_t kBroadcastId = 0xFE;

// STS3215 register addresses. EEPROM registers (below 40) keep their value without power
constexpr uint8_t kModel = 3;             // u16, 777 for an STS3215
constexpr uint8_t kId = 5;                // u8
constexpr uint8_t kReturnDelay = 7;       // u8, wait before replying, in 2 us steps
constexpr uint8_t kTorqueEnable = 40;     // u8
constexpr uint8_t kGoalPosition = 42;     // u16
constexpr uint8_t kLock = 55;             // u8, 0 = EEPROM writable
constexpr uint8_t kPresentPosition = 56;  // start of the state block, see decode_state()
constexpr uint8_t kStateSize = 8;
constexpr uint8_t kStatus = 65;           // u8 error flags, 0 = no error

constexpr int kStepsPerTurn = 4096;  // positions go 0-4095 over 360 degrees
constexpr int kCenterStep = 2048;    // joint angle 0 before calibration

// Longest packet: a sync write of 2 bytes to 253 servos
constexpr size_t kMaxPacket = 4 + 2 + 2 + 253 * 3 + 1;

struct State
{
  int position;     // steps, 0-4095
  int speed;        // steps/s
  int load;         // per mille of the maximum PWM, signed
  double voltage;   // V
  int temperature;  // degrees C
};

uint8_t checksum(const uint8_t * bytes, size_t size);

// Writes a request into out, which keeps its capacity: no allocation once it is reserved
void make_packet(
  std::vector<uint8_t> & out, uint8_t id, uint8_t instruction, const uint8_t * params,
  size_t size);

// Feetech stores signed values as a magnitude plus one sign bit
int decode_signed(uint16_t value, int sign_bit);

// The 8-byte block at kPresentPosition
State decode_state(const uint8_t * data);

}  // namespace h3215_hardware::sts

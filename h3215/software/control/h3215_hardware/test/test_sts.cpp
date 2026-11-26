// The protocol and the bus against simulated servos on a pseudo-terminal (no hardware needed).

#include <gtest/gtest.h>
#include <pty.h>
#include <unistd.h>

#include <atomic>
#include <map>
#include <thread>
#include <vector>

#include "h3215_hardware/sts_bus.hpp"
#include "h3215_hardware/sts_protocol.hpp"

using h3215_hardware::StsBus;
namespace sts = h3215_hardware::sts;

namespace
{

// Answers like STS servos with the given memory, on the other end of a pseudo-terminal
class FakeServos
{
public:
  explicit FakeServos(std::map<uint8_t, std::vector<uint8_t>> memory)
  : memory_(std::move(memory))
  {
    char name[64];
    EXPECT_EQ(openpty(&master_, &slave_, name, nullptr, nullptr), 0);
    path_ = name;
    thread_ = std::thread([this] { run(); });
  }

  ~FakeServos()
  {
    stop_ = true;
    thread_.join();
    close(slave_);
    close(master_);
  }

  const std::string & path() const { return path_; }
  std::vector<uint8_t> & memory(uint8_t id) { return memory_.at(id); }
  std::atomic<bool> corrupt_next{false};
  std::atomic<int> mute_id{-1};

private:
  void run()
  {
    std::vector<uint8_t> rx;
    while (!stop_) {
      uint8_t buf[512];
      fd_set set;
      FD_ZERO(&set);
      FD_SET(master_, &set);
      timeval tv{0, 2000};
      if (select(master_ + 1, &set, nullptr, nullptr, &tv) <= 0) {
        continue;
      }
      const ssize_t n = read(master_, buf, sizeof(buf));
      rx.insert(rx.end(), buf, buf + std::max<ssize_t>(n, 0));
      while (rx.size() >= 4 && rx.size() >= 4u + rx[3]) {
        std::vector<uint8_t> packet(rx.begin(), rx.begin() + 4 + rx[3]);
        rx.erase(rx.begin(), rx.begin() + 4 + rx[3]);
        EXPECT_EQ(packet.back(), sts::checksum(packet.data() + 2, packet.size() - 3));
        handle(packet);
      }
    }
  }

  void handle(const std::vector<uint8_t> & p)
  {
    const uint8_t id = p[2], instruction = p[4];
    const std::vector<uint8_t> params(p.begin() + 5, p.end() - 1);
    if (instruction == sts::kPing && memory_.count(id)) {
      reply(id, {});
    } else if (instruction == sts::kRead && memory_.count(id)) {
      auto & m = memory_[id];
      reply(id, {m.begin() + params[0], m.begin() + params[0] + params[1]});
    } else if (instruction == sts::kWrite && memory_.count(id)) {
      std::copy(params.begin() + 1, params.end(), memory_[id].begin() + params[0]);
      reply(id, {});
    } else if (instruction == sts::kSyncRead) {
      for (size_t k = 2; k < params.size(); ++k) {
        const uint8_t sid = params[k];
        if (memory_.count(sid) && sid != mute_id) {
          auto & m = memory_[sid];
          reply(sid, {m.begin() + params[0], m.begin() + params[0] + params[1]});
        }
      }
    } else if (instruction == sts::kSyncWrite) {
      const uint8_t address = params[0], size = params[1];
      for (size_t k = 2; k + size < params.size() + 1; k += size + 1) {
        std::copy(
          params.begin() + k + 1, params.begin() + k + 1 + size,
          memory_[params[k]].begin() + address);
      }
    }
  }

  void reply(uint8_t id, const std::vector<uint8_t> & data)
  {
    std::vector<uint8_t> body = {id, static_cast<uint8_t>(data.size() + 2), 0};
    body.insert(body.end(), data.begin(), data.end());
    std::vector<uint8_t> out = {0xFF, 0xFF};
    out.insert(out.end(), body.begin(), body.end());
    out.push_back(sts::checksum(body.data(), body.size()));
    if (corrupt_next.exchange(false)) {
      out.back() ^= 0xFF;
    }
    EXPECT_EQ(::write(master_, out.data(), out.size()), static_cast<ssize_t>(out.size()));
  }

  std::map<uint8_t, std::vector<uint8_t>> memory_;
  int master_ = -1, slave_ = -1;
  std::string path_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

std::map<uint8_t, std::vector<uint8_t>> servos()
{
  std::map<uint8_t, std::vector<uint8_t>> memory = {{1, std::vector<uint8_t>(70)}, {2, std::vector<uint8_t>(70)}};
  // position 2048, speed -100, load -50 (sign-magnitude), 12.0 V, 35 C
  const uint8_t state[] = {0x00, 0x08, 100, 0x80, 50, 0x04, 120, 35};
  std::copy(state, state + 8, memory[1].begin() + sts::kPresentPosition);
  return memory;
}

}  // namespace

TEST(StsProtocol, PingPacket)
{
  std::vector<uint8_t> out;
  sts::make_packet(out, 1, sts::kPing, nullptr, 0);
  EXPECT_EQ(out, (std::vector<uint8_t>{0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB}));
}

TEST(StsProtocol, DecodeState)
{
  const uint8_t data[] = {0x00, 0x08, 100, 0x80, 50, 0x04, 120, 35};
  const sts::State s = sts::decode_state(data);
  EXPECT_EQ(s.position, 2048);
  EXPECT_EQ(s.speed, -100);
  EXPECT_EQ(s.load, -50);
  EXPECT_DOUBLE_EQ(s.voltage, 12.0);
  EXPECT_EQ(s.temperature, 35);
}

TEST(StsBus, PingAndRead)
{
  FakeServos fake(servos());
  StsBus bus;
  ASSERT_EQ(bus.open(fake.path(), 1000000), "");
  EXPECT_TRUE(bus.ping(1));
  EXPECT_TRUE(bus.ping(2));
  EXPECT_FALSE(bus.ping(3));
  uint8_t state[sts::kStateSize];
  ASSERT_TRUE(bus.read(1, sts::kPresentPosition, sts::kStateSize, state));
  EXPECT_EQ(sts::decode_state(state).position, 2048);
}

TEST(StsBus, SyncWriteThenSyncRead)
{
  FakeServos fake(servos());
  StsBus bus;
  ASSERT_EQ(bus.open(fake.path(), 1000000), "");
  const std::vector<uint8_t> ids = {1, 2};
  const uint8_t goals[] = {0xE8, 0x03, 0xB8, 0x0B};  // 1000, 3000
  ASSERT_TRUE(bus.sync_write(ids, sts::kGoalPosition, 2, goals));
  usleep(5000);  // sync writes get no reply: give the fake time to apply it
  uint8_t out[4], errors[2];
  bool received[2];
  EXPECT_EQ(bus.sync_read(ids, sts::kGoalPosition, 2, out, errors, received), 2u);
  EXPECT_EQ(std::vector<uint8_t>(out, out + 4), std::vector<uint8_t>(goals, goals + 4));
}

TEST(StsBus, SyncReadWithMissingServo)
{
  FakeServos fake(servos());
  StsBus bus;
  ASSERT_EQ(bus.open(fake.path(), 1000000), "");
  bus.set_timeout(std::chrono::milliseconds(20));
  fake.mute_id = 1;
  const std::vector<uint8_t> ids = {1, 2};
  uint8_t out[2 * sts::kStateSize], errors[2];
  bool received[2];
  EXPECT_EQ(bus.sync_read(ids, sts::kPresentPosition, sts::kStateSize, out, errors, received), 1u);
  EXPECT_FALSE(received[0]);
  EXPECT_TRUE(received[1]);
}

TEST(StsBus, WriteAndCorruptedReply)
{
  FakeServos fake(servos());
  StsBus bus;
  ASSERT_EQ(bus.open(fake.path(), 1000000), "");
  const uint8_t on = 1;
  ASSERT_TRUE(bus.write(2, sts::kTorqueEnable, &on, 1));
  EXPECT_EQ(fake.memory(2)[sts::kTorqueEnable], 1);
  fake.corrupt_next = true;
  EXPECT_FALSE(bus.ping(1));
  EXPECT_TRUE(bus.ping(1));
}

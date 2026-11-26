// Servo setup and checks, on the bus alone: stop ros2_control first, only one program at a
// time can own the port.
//
//   ros2 run h3215_hardware servo_tool scan             list the servos and their settings
//   ros2 run h3215_hardware servo_tool set-id 1 2       give the servo with ID 1 the ID 2
//   ros2 run h3215_hardware servo_tool fast-reply       make every servo reply without delay
//   ros2 run h3215_hardware servo_tool check            status and bus timing, nothing moves
//   ros2 run h3215_hardware servo_tool check --move     then a slow sine on every servo
//
// Options: --port <path> (default /dev/ttyACM0); for check: --ids <id>..., --amplitude <deg>
// (default 10), --rate <Hz> (default 50), --seconds <s> (default 10).
//
// New servos all ship with ID 1: connect only the new servo when you change its ID, otherwise
// every servo with ID 1 answers at once. If scan shows a reply delay above 0, run fast-reply:
// at 500 us per servo, 18 servos would add 9 ms to every read.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include "h3215_hardware/sts_bus.hpp"
#include "h3215_hardware/sts_protocol.hpp"

using h3215_hardware::StsBus;
namespace sts = h3215_hardware::sts;
using Clock = std::chrono::steady_clock;

namespace
{

constexpr int kBitsPerByte = 10;  // 8 data bits + start bit + stop bit
volatile std::sig_atomic_t g_stop = 0;

struct Options
{
  std::string command;
  std::vector<std::string> positional;
  std::string port = "/dev/ttyACM0";
  std::vector<uint8_t> ids;
  bool move = false;
  double amplitude = 10.0;
  double rate = 50.0;
  double seconds = 10.0;
};

[[noreturn]] void fail(const std::string & message)
{
  std::fprintf(stderr, "%s\n", message.c_str());
  std::exit(1);
}

Options parse(int argc, char ** argv)
{
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto value = [&]() -> std::string {
      if (i + 1 >= argc) {
        fail("missing value after " + a);
      }
      return argv[++i];
    };
    if (a == "--port") {
      o.port = value();
    } else if (a == "--move") {
      o.move = true;
    } else if (a == "--amplitude") {
      o.amplitude = std::stod(value());
    } else if (a == "--rate") {
      o.rate = std::stod(value());
    } else if (a == "--seconds") {
      o.seconds = std::stod(value());
    } else if (a == "--ids") {
      while (i + 1 < argc && argv[i + 1][0] != '-') {
        o.ids.push_back(static_cast<uint8_t>(std::stoi(argv[++i])));
      }
    } else if (a == "-h" || a == "--help") {
      o.command = "help";
    } else if (o.command.empty()) {
      o.command = a;
    } else {
      o.positional.push_back(a);
    }
  }
  return o;
}

int read_u8(StsBus & bus, uint8_t id, uint8_t address)
{
  uint8_t v = 0;
  if (!bus.read(id, address, 1, &v)) {
    fail("no reply from servo " + std::to_string(id));
  }
  return v;
}

int read_u16(StsBus & bus, uint8_t id, uint8_t address)
{
  uint8_t v[2] = {0, 0};
  if (!bus.read(id, address, 2, v)) {
    fail("no reply from servo " + std::to_string(id));
  }
  return v[0] | (v[1] << 8);
}

void write_u8(StsBus & bus, uint8_t id, uint8_t address, uint8_t value)
{
  if (!bus.write(id, address, &value, 1)) {
    fail("servo " + std::to_string(id) + " didn't confirm a write");
  }
}

// EEPROM is write-protected: unlock, write, lock again
void write_eeprom(StsBus & bus, uint8_t id, uint8_t address, uint8_t value)
{
  write_u8(bus, id, sts::kLock, 0);
  write_u8(bus, id, address, value);
  write_u8(bus, address == sts::kId ? value : id, sts::kLock, 1);
}

double degrees(int steps) { return steps * 360.0 / sts::kStepsPerTurn; }

std::vector<uint8_t> found(StsBus & bus)
{
  const auto ids = bus.scan();
  if (ids.empty()) {
    fail(
      "No servos found: check the servo power supply, the cable, the port (--port), and "
      "that the board's jumpers are on B (USB).");
  }
  return ids;
}

void print_settings(StsBus & bus, const std::vector<uint8_t> & ids)
{
  std::printf(" id  model  reply delay  position\n");
  for (const uint8_t id : ids) {
    std::printf(
      "%3d  %5d  %8d us  %6.1f deg\n", id, read_u16(bus, id, sts::kModel),
      read_u8(bus, id, sts::kReturnDelay) * 2, degrees(read_u16(bus, id, sts::kPresentPosition)));
  }
}

struct Stats
{
  double median, p99, max;
};

Stats stats(std::vector<double> v)
{
  std::sort(v.begin(), v.end());
  const auto at = [&](double q) { return v[static_cast<size_t>(q * (v.size() - 1))]; };
  return {at(0.5), at(0.99), v.back()};
}

double ms_since(Clock::time_point t0)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

bool sync_read_state(StsBus & bus, const std::vector<uint8_t> & ids, std::vector<sts::State> & out)
{
  std::vector<uint8_t> bytes(ids.size() * sts::kStateSize), errors(ids.size());
  std::unique_ptr<bool[]> received(new bool[ids.size()]);
  const size_t n = bus.sync_read(
    ids, sts::kPresentPosition, sts::kStateSize, bytes.data(), errors.data(), received.get());
  out.resize(ids.size());
  for (size_t k = 0; k < ids.size(); ++k) {
    if (received[k]) {
      out[k] = sts::decode_state(bytes.data() + k * sts::kStateSize);
    }
  }
  return n == ids.size();
}

bool sync_write_goals(StsBus & bus, const std::vector<uint8_t> & ids, const std::vector<int> & goals)
{
  std::vector<uint8_t> data(2 * ids.size());
  for (size_t k = 0; k < ids.size(); ++k) {
    data[2 * k] = static_cast<uint8_t>(goals[k] & 0xFF);
    data[2 * k + 1] = static_cast<uint8_t>(goals[k] >> 8);
  }
  return bus.sync_write(ids, sts::kGoalPosition, 2, data.data());
}

void set_torque(StsBus & bus, const std::vector<uint8_t> & ids, bool on)
{
  std::vector<uint8_t> values(ids.size(), on ? 1 : 0);
  bus.sync_write(ids, sts::kTorqueEnable, 1, values.data());
}

void status(StsBus & bus, const std::vector<uint8_t> & ids)
{
  std::printf(" id  position   speed    load  voltage  temp  reply delay  errors\n");
  for (const uint8_t id : ids) {
    uint8_t block[sts::kStateSize];
    if (!bus.read(id, sts::kPresentPosition, sts::kStateSize, block)) {
      fail("no reply from servo " + std::to_string(id));
    }
    const sts::State s = sts::decode_state(block);
    std::printf(
      "%3d  %5.1f deg  %6d  %5.1f %%  %5.1f V  %2d C  %8d us    0x%02x\n", id, degrees(s.position),
      s.speed, s.load / 10.0, s.voltage, s.temperature, read_u8(bus, id, sts::kReturnDelay) * 2,
      read_u8(bus, id, sts::kStatus));
  }
}

void timing(StsBus & bus, const std::vector<uint8_t> & ids)
{
  std::vector<double> counts, medians;
  std::vector<sts::State> state;
  for (size_t k = 1; k <= ids.size(); ++k) {
    const std::vector<uint8_t> subset(ids.begin(), ids.begin() + k);
    std::vector<double> t;
    for (int i = 0; i < 300; ++i) {
      const auto t0 = Clock::now();
      sync_read_state(bus, subset, state);
      t.push_back(ms_since(t0));
    }
    const Stats s = stats(t);
    counts.push_back(k);
    medians.push_back(s.median);
    std::printf(
      "  sync-read, %zu servo(s): median %.2f  p99 %.2f  max %.2f ms\n", k, s.median, s.p99, s.max);
  }
  // Read time = fixed cost + cost per servo (least squares). With one servo, count it all as per servo
  double per_servo = medians[0], fixed = 0.0;
  if (counts.size() > 1) {
    const double n = counts.size();
    const double mx = std::accumulate(counts.begin(), counts.end(), 0.0) / n;
    const double my = std::accumulate(medians.begin(), medians.end(), 0.0) / n;
    double sxy = 0, sxx = 0;
    for (size_t i = 0; i < counts.size(); ++i) {
      sxy += (counts[i] - mx) * (medians[i] - my);
      sxx += (counts[i] - mx) * (counts[i] - mx);
    }
    per_servo = sxy / sxx;
    fixed = my - per_servo * mx;
  }
  const double read_18 = fixed + 18 * per_servo;
  const double write_18 = (8 + 18 * 3) * kBitsPerByte * 1000.0 / bus.baudrate();  // no reply
  std::printf("  per servo %.3f ms, fixed %.3f ms\n", per_servo, fixed);
  std::printf(
    "  projected 18-servo cycle: %.2f write + %.2f read = %.2f ms\n", write_18, read_18,
    write_18 + read_18);
}

// Slow sine around the current positions; reports bus time and tracking error
void move(StsBus & bus, const std::vector<uint8_t> & ids, const Options & o)
{
  std::vector<sts::State> state;
  if (!sync_read_state(bus, ids, state)) {
    fail("cannot read every servo");
  }
  std::vector<int> start(ids.size());
  for (size_t k = 0; k < ids.size(); ++k) {
    start[k] = state[k].position;
  }
  sync_write_goals(bus, ids, start);  // goal = here...
  set_torque(bus, ids, true);         // ...then power on

  std::signal(SIGINT, [](int) { g_stop = 1; });
  const double amplitude = o.amplitude * sts::kStepsPerTurn / 360.0;
  const int n = static_cast<int>(o.seconds * o.rate);
  const auto period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / o.rate));
  std::vector<double> io_ms, error_deg;
  std::vector<int> goal(ids.size());
  auto next = Clock::now();
  for (int k = 0; k < n && !g_stop; ++k) {
    const double offset = amplitude * std::sin(2 * M_PI * 0.5 * k / o.rate);
    for (size_t j = 0; j < ids.size(); ++j) {
      goal[j] = std::clamp(static_cast<int>(std::lround(start[j] + offset)), 0, sts::kStepsPerTurn - 1);
    }
    const auto t0 = Clock::now();
    sync_write_goals(bus, ids, goal);
    sync_read_state(bus, ids, state);
    io_ms.push_back(ms_since(t0));
    int worst = 0;
    for (size_t j = 0; j < ids.size(); ++j) {
      worst = std::max(worst, std::abs(state[j].position - goal[j]));
    }
    error_deg.push_back(degrees(worst));
    next += period;
    std::this_thread::sleep_until(next);
  }
  // Also on Ctrl+C: back to the start, then let go
  sync_write_goals(bus, ids, start);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  set_torque(bus, ids, false);
  if (io_ms.empty()) {
    return;
  }
  const Stats io = stats(io_ms), err = stats(error_deg);
  std::printf(
    "  bus cycle (write + read): median %.2f  p99 %.2f  max %.2f ms\n", io.median, io.p99, io.max);
  std::printf(
    "  tracking error (includes the servo's lag): median %.2f  max %.2f deg\n", err.median, err.max);
}

void usage()
{
  std::printf(
    "usage: servo_tool [--port PATH] scan | set-id OLD NEW | fast-reply\n"
    "       servo_tool [--port PATH] check [--ids ID...] [--move] [--amplitude DEG] [--rate HZ] "
    "[--seconds S]\n");
}

}  // namespace

int main(int argc, char ** argv)
{
  const Options o = parse(argc, argv);
  if (o.command.empty() || o.command == "help") {
    usage();
    return o.command.empty() ? 1 : 0;
  }
  StsBus bus;
  const std::string error = bus.open(o.port, 1000000);
  if (!error.empty()) {
    fail(error);
  }
  bus.set_timeout(std::chrono::milliseconds(20));

  if (o.command == "scan") {
    print_settings(bus, found(bus));
  } else if (o.command == "set-id") {
    if (o.positional.size() != 2) {
      fail("usage: servo_tool set-id OLD NEW");
    }
    const int old_id = std::stoi(o.positional[0]), new_id = std::stoi(o.positional[1]);
    if (new_id < 0 || new_id >= sts::kBroadcastId) {
      fail("IDs go from 0 to " + std::to_string(sts::kBroadcastId - 1));
    }
    if (!bus.ping(old_id)) {
      fail("No servo answers as ID " + std::to_string(old_id));
    }
    if (bus.ping(new_id)) {
      fail("ID " + std::to_string(new_id) + " is already taken");
    }
    write_eeprom(bus, old_id, sts::kId, new_id);
    print_settings(bus, found(bus));
  } else if (o.command == "fast-reply") {
    const auto ids = found(bus);
    for (const uint8_t id : ids) {
      if (read_u8(bus, id, sts::kReturnDelay) != 0) {
        write_eeprom(bus, id, sts::kReturnDelay, 0);
      }
    }
    print_settings(bus, ids);
  } else if (o.command == "check") {
    const auto ids = o.ids.empty() ? found(bus) : o.ids;
    std::printf("Servos on %s:", o.port.c_str());
    for (const uint8_t id : ids) {
      std::printf(" %d", id);
    }
    std::printf("\n\n");
    status(bus, ids);
    std::printf("\nBus timing\n");
    timing(bus, ids);
    if (o.move) {
      std::printf(
        "\nSine motion, +/-%g deg at 0.5 Hz, %g Hz control, %g s\n", o.amplitude, o.rate, o.seconds);
      move(bus, ids, o);
    }
  } else {
    usage();
    return 1;
  }
  return 0;
}

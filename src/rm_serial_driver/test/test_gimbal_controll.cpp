#include <gtest/gtest.h>

#include <mutex>
#include <thread>

#include "linux_uart.hpp"
#include "rm_serial_driver/SharedTopic.hpp"
#include "rm_serial_driver/SharedTopicClient.hpp"

using namespace std::chrono_literals;

namespace
{

enum class Mode : uint8_t
{
  UP = 0,
  DOWN = 1,
  PINGPONG = 2,
  FIXED = 3
};

struct SweepIndex
{
  std::size_t idx = 0;
  int dir = +1;
};

struct TestConfig
{
  float pitch_min = -0.3f;
  float pitch_max = 0.3f;
  int pitch_steps = 10;

  float yaw_min = -0.5f;
  float yaw_max = 0.5f;
  int yaw_steps = 10;

  int switch_period = 250;
  int max_switches = 20;
#include <thread>

  float pitch_tolerance = 0.03f;
  float yaw_tolerance = 0.03f;

  int control_period_ms = 2;
  int pose_wait_timeout_ms = 5000;

  Mode pitch_mode = Mode::PINGPONG;
  Mode yaw_mode = Mode::PINGPONG;
};

std::string to_lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

bool parse_mode(const std::string& text, Mode& out_mode)
{
  static const std::pair<const char*, Mode> K_TABLE[] = {
      {"0", Mode::UP},
      {"up", Mode::UP},
      {"1", Mode::DOWN},
      {"down", Mode::DOWN},
      {"2", Mode::PINGPONG},
      {"pingpong", Mode::PINGPONG},
      {"ping_pong", Mode::PINGPONG},
      {"3", Mode::FIXED},
      {"fixed", Mode::FIXED},
  };
  std::string s = to_lower(text);
  for (const auto& [key, mode] : K_TABLE)
  {
    if (s == key)
    {
      out_mode = mode;
      return true;
    }
  }
  return false;
}

template <typename T>
bool consume_number_arg(const std::string& arg, const std::string& key, T& value)
{
  if (arg.rfind(key, 0) != 0)
  {
    return false;
  }
  std::istringstream iss(arg.substr(key.size()));
  T tmp{};
  iss >> tmp;
  if (iss.fail() || !iss.eof())
  {
    return false;
  }
  value = tmp;
  return true;
}

bool consume_mode_arg(const std::string& arg, const std::string& key, Mode& mode)
{
  if (arg.rfind(key, 0) != 0)
  {
    return false;
  }
  return parse_mode(arg.substr(key.size()), mode);
}

bool consume_custom_arg(const std::string& arg, TestConfig& cfg)
{
  return consume_number_arg(arg, "--pitch_min=", cfg.pitch_min) ||
         consume_number_arg(arg, "--pitch_max=", cfg.pitch_max) ||
         consume_number_arg(arg, "--pitch_steps=", cfg.pitch_steps) ||
         consume_number_arg(arg, "--yaw_min=", cfg.yaw_min) ||
         consume_number_arg(arg, "--yaw_max=", cfg.yaw_max) ||
         consume_number_arg(arg, "--yaw_steps=", cfg.yaw_steps) ||
         consume_number_arg(arg, "--switch_period=", cfg.switch_period) ||
         consume_number_arg(arg, "--max_switches=", cfg.max_switches) ||
         consume_number_arg(arg, "--pitch_tolerance=", cfg.pitch_tolerance) ||
         consume_number_arg(arg, "--yaw_tolerance=", cfg.yaw_tolerance) ||
         consume_number_arg(arg, "--control_period_ms=", cfg.control_period_ms) ||
         consume_number_arg(arg, "--pose_wait_timeout_ms=", cfg.pose_wait_timeout_ms) ||
         consume_mode_arg(arg, "--pitch_mode=", cfg.pitch_mode) ||
         consume_mode_arg(arg, "--yaw_mode=", cfg.yaw_mode);
}

void parse_custom_args(int* argc, char** argv, TestConfig& cfg)
{
  int write_idx = 1;
  for (int i = 1; i < *argc; ++i)
  {
    std::string arg(argv[i]);
    if (consume_custom_arg(arg, cfg))
    {
      continue;
    }
    argv[write_idx++] = argv[i];
  }

  *argc = write_idx;
  argv[write_idx] = nullptr;
}

void normalize_config(TestConfig& cfg)
{
  auto fix = [](auto& v, auto fallback)
  {
    if (v <= std::remove_reference_t<decltype(v)>{})
    {
      v = fallback;
    }
  };
  fix(cfg.pitch_steps, 1);
  if (cfg.yaw_steps <= 0)
  {
    cfg.yaw_steps = cfg.pitch_steps;
  }
  fix(cfg.switch_period, 250);
  fix(cfg.max_switches, 20);
  fix(cfg.control_period_ms, 2);
  fix(cfg.pose_wait_timeout_ms, 5000);
  fix(cfg.pitch_tolerance, 0.03f);
  fix(cfg.yaw_tolerance, 0.03f);
}

std::vector<float> generate_sequence(std::pair<float, float> limit, std::size_t step,
                                     Mode mode)
{
  const float LOW = std::min(limit.first, limit.second);
  const float HIGH = std::max(limit.first, limit.second);
  const std::size_t N = std::max<std::size_t>(step, 1);

  if (N == 1 || mode == Mode::FIXED)
  {
    return {(HIGH + LOW) * 0.5f};
  }

  std::vector<float> table(N);
  for (std::size_t i = 0; i < N; ++i)
  {
    table[i] = LOW + (HIGH - LOW) * static_cast<float>(i) / static_cast<float>(N - 1);
  }

  if (mode == Mode::DOWN)
  {
    std::reverse(table.begin(), table.end());
  }
  return table;
}

void advance_index(SweepIndex& index, std::size_t size, Mode mode)
{
  if (size <= 1 || mode == Mode::FIXED)
  {
    return;
  }

  if (mode == Mode::UP || mode == Mode::DOWN)
  {
    index.idx = (index.idx + 1) % size;
    return;
  }

  if (index.dir > 0 && index.idx + 1 >= size)
  {
    index.dir = -1;
  }
  else if (index.dir < 0 && index.idx == 0)
  {
    index.dir = +1;
  }
  index.idx = static_cast<std::size_t>(abs(static_cast<int>(index.idx) + index.dir));
}

double normalize_angle(double angle) { return std::remainder(angle, 2.0 * M_PI); }

class PoseReceiver
{
 public:
  void Reset() { has_pose_.store(false, std::memory_order_release); }

  void Update(const LibXR::Quaternion<float>& quat)
  {
    LibXR::EulerAngle<float> euler = quat.ToEulerAngleZYX();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pitch_ = static_cast<double>(euler.Pitch());
      yaw_ = static_cast<double>(euler.Yaw());
    }

    has_pose_.store(true, std::memory_order_release);
  }

  bool HasPose() const { return has_pose_.load(std::memory_order_acquire); }

  std::pair<double, double> Read() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return {pitch_, yaw_};
  }

 private:
  std::atomic<bool> has_pose_{false};
  mutable std::mutex mutex_;
  double pitch_ = 0.0;
  double yaw_ = 0.0;
};

PoseReceiver& get_pose_receiver()
{
  static PoseReceiver receiver;
  return receiver;
}

struct TransportContext
{
  std::unique_ptr<LibXR::RamFS> ramfs;
  std::unique_ptr<LibXR::LinuxUART> uart;
  LibXR::HardwareContainer* hw = nullptr;
};

TransportContext& get_transport_context()
{
  static TransportContext ctx;
  return ctx;
}

LibXR::HardwareContainer& init_transport_once()
{
  auto& ctx = get_transport_context();
  if (ctx.hw != nullptr)
  {
    return *ctx.hw;
  }

  LibXR::PlatformInit();

  ctx.ramfs = std::make_unique<LibXR::RamFS>();
  ctx.uart = std::make_unique<LibXR::LinuxUART>(
      "16d0", "1492", 115200, LibXR::LinuxUART::Parity::NO_PARITY, 8, 1);

  static LibXR::HardwareContainer peripherals{
      LibXR::Entry<LibXR::RamFS>({*ctx.ramfs, {"ramfs"}}),
      LibXR::Entry<LibXR::UART>({*ctx.uart, {"uart_client"}}),
  };

  ctx.hw = &peripherals;
  return *ctx.hw;
}

void start_shared_topic_apps_once(LibXR::HardwareContainer& hw)
{
  static LibXR::ApplicationManager appmgr;
  static SharedTopic shared_topic(hw, appmgr, "uart_client", 256, {{"ahrs_quaternion"}});
  static SharedTopicClient shared_topic_client(hw, appmgr, "uart_client", 256,
                                               {{"target_euler"}});
}

bool wait_for_pose(int timeout_ms)
{
  auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() < deadline)
  {
    if (get_pose_receiver().HasPose())
    {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }

  return get_pose_receiver().HasPose();
}

TestConfig& global_config()
{
  static TestConfig cfg;
  return cfg;
}

class GimbalControllTest : public ::testing::Test
{
 public:
  void SetUp() override
  {
    LibXR::PlatformInit();
    ahrs_quaternion_topic_ =
        LibXR::Topic::FindOrCreate<LibXR::Quaternion<float>>("ahrs_quaternion");
    target_euler_topic_ =
        LibXR::Topic::FindOrCreate<LibXR::EulerAngle<float>>("target_euler");

    static LibXR::RamFS ramfs;
    static LibXR::LinuxUART uart("16d0", "1492", 115200,
                                 LibXR::LinuxUART::Parity::NO_PARITY, 8, 1);

    static LibXR::HardwareContainer peripherals{
        LibXR::Entry<LibXR::RamFS>({ramfs, {"ramfs"}}),
        LibXR::Entry<LibXR::UART>({uart, {"uart_client"}}),
    };

    static LibXR::ApplicationManager appmgr;
    static SharedTopic shared_topic(peripherals, appmgr, "uart_client", 256,
                                    {{"ahrs_quaternion"}});
    static SharedTopicClient shared_topic_client(peripherals, appmgr, "uart_client", 256,
                                                 {{"target_euler"}});

    void (*cb_fun)(bool, PoseReceiver*, LibXR::RawData&) =
        [](bool, PoseReceiver* receiver, LibXR::RawData& data)
    {
      auto* quat = reinterpret_cast<LibXR::Quaternion<float>*>(data.addr_);
      receiver->Update(*quat);
    };

    auto cb = LibXR::Topic::Callback::Create(cb_fun, &get_pose_receiver());
    ahrs_quaternion_topic_.RegisterCallback(cb);

    cfg_ = global_config();
    normalize_config(cfg_);

    pitch_table_ =
        generate_sequence({cfg_.pitch_min, cfg_.pitch_max},
                          static_cast<std::size_t>(cfg_.pitch_steps), cfg_.pitch_mode);
    yaw_table_ =
        generate_sequence({cfg_.yaw_min, cfg_.yaw_max},
                          static_cast<std::size_t>(cfg_.yaw_steps), cfg_.yaw_mode);
    UpdateCurrentTarget();

    get_pose_receiver().Reset();
  }

  void UpdateCurrentTarget()
  {
    current_pitch_target_ =
        pitch_table_.empty() ? 0.0 : static_cast<double>(pitch_table_[pitch_index_.idx]);
    current_yaw_target_ =
        yaw_table_.empty() ? 0.0 : static_cast<double>(yaw_table_[yaw_index_.idx]);
  }

  void PublishCurrentTarget()
  {
    LibXR::EulerAngle<float> target{};
    target.Pitch() = static_cast<float>(current_pitch_target_);
    target.Yaw() = static_cast<float>(current_yaw_target_);
    target.Roll() = 0.0f;
    target_euler_topic_.Publish(target);
  }

  std::pair<double, double> CurrentPose() const { return get_pose_receiver().Read(); }

  TestConfig cfg_;

  std::vector<float> pitch_table_;
  std::vector<float> yaw_table_;
  SweepIndex pitch_index_;
  SweepIndex yaw_index_;

  double current_pitch_target_ = 0.0;
  double current_yaw_target_ = 0.0;

  LibXR::Topic ahrs_quaternion_topic_;
  LibXR::Topic target_euler_topic_;
};

TEST_F(GimbalControllTest, CheckPoseErrorBeforeNextTarget)
{
  ASSERT_TRUE(wait_for_pose(cfg_.pose_wait_timeout_ms))
      << "No ahrs_quaternion received from lower machine within timeout.";

  int loop_count = 0;
  int checked_switches = 0;

  while (checked_switches < cfg_.max_switches)
  {
    PublishCurrentTarget();
    std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.control_period_ms));

    if (++loop_count % cfg_.switch_period != 0)
    {
      continue;
    }

    ASSERT_TRUE(get_pose_receiver().HasPose())
        << "Lost gimbal pose before switch #" << checked_switches;

    const auto [pitch_now, yaw_now] = CurrentPose();
    double pitch_error = std::abs(normalize_angle(pitch_now - current_pitch_target_));
    double yaw_error = std::abs(normalize_angle(yaw_now - current_yaw_target_));

    EXPECT_LE(pitch_error, static_cast<double>(cfg_.pitch_tolerance))
        << "switch #" << checked_switches
        << " pitch error too large. target=" << current_pitch_target_
        << ", current=" << pitch_now << ", tolerance=" << cfg_.pitch_tolerance;

    EXPECT_LE(yaw_error, static_cast<double>(cfg_.yaw_tolerance))
        << "switch #" << checked_switches
        << " yaw error too large. target=" << current_yaw_target_
        << ", current=" << yaw_now << ", tolerance=" << cfg_.yaw_tolerance;

    advance_index(pitch_index_, pitch_table_.size(), cfg_.pitch_mode);
    advance_index(yaw_index_, yaw_table_.size(), cfg_.yaw_mode);
    UpdateCurrentTarget();

    ++checked_switches;
  }

  ASSERT_EQ(checked_switches, cfg_.max_switches);
}

}  // namespace

int main(int argc, char** argv)
{
  parse_custom_args(&argc, argv, global_config());
  normalize_config(global_config());

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

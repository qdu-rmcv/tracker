#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <numeric>
#include <sstream>
#include <thread>
#include <vector>

#include "linux_uart.hpp"
#include "rm_serial_driver/SharedTopic.hpp"
#include "rm_serial_driver/SharedTopicClient.hpp"

using namespace std::chrono_literals;

namespace
{

// ---- Wire Protocol Structures ----------------------------------------

struct TsPing
{
  uint32_t sequence;
};

struct TsPong
{
  uint32_t sequence;
  uint64_t mcu_timestamp;
};

struct AhrsStamped
{
  LibXR::Quaternion<float> quaternion;
  uint64_t mcu_timestamp;
};

// ---- Configuration ---------------------------------------------------

struct TestConfig
{
  std::string vid = "16d0";
  std::string pid = "1492";
  int rtt_sample_count = 100;
  int rtt_interval_ms = 10;
  int delay_sample_count = 500;
  int delay_sample_timeout_s = 10;
};

// ---- CLI Argument Parsing --------------------------------------------

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

bool consume_string_arg(const std::string& arg, const std::string& key,
                        std::string& value)
{
  if (arg.rfind(key, 0) != 0)
  {
    return false;
  }
  value = arg.substr(key.size());
  return !value.empty();
}

bool consume_custom_arg(const std::string& arg, TestConfig& cfg)
{
  return consume_string_arg(arg, "--vid=", cfg.vid) ||
         consume_string_arg(arg, "--pid=", cfg.pid) ||
         consume_number_arg(arg, "--rtt_sample_count=", cfg.rtt_sample_count) ||
         consume_number_arg(arg, "--rtt_interval_ms=", cfg.rtt_interval_ms) ||
         consume_number_arg(arg, "--delay_sample_count=", cfg.delay_sample_count) ||
         consume_number_arg(arg, "--delay_sample_timeout_s=", cfg.delay_sample_timeout_s);
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

TestConfig& global_config()
{
  static TestConfig cfg;
  return cfg;
}

// ---- Helpers ---------------------------------------------------------

double now_microseconds()
{
  auto now = std::chrono::steady_clock::now();
  return std::chrono::duration<double, std::micro>(now.time_since_epoch()).count();
}

// ---- Test Fixture ----------------------------------------------------

class TimestampCalibratorTest : public ::testing::Test
{
 public:
  struct RttSample
  {
    uint32_t sequence;
    double send_time_us;
    double recv_time_us;
    double rtt_us;
  };

  struct DelaySample
  {
    double pc_recv_time_us;
    uint64_t mcu_timestamp;
    double estimated_delay_us;
  };

  struct ClockSample
  {
    double pc_send;
    double rtt;
    uint64_t mcu_ts;
  };

  void SetUp() override
  {
    cfg_ = global_config();

    LibXR::PlatformInit();

    ts_ping_topic_ = LibXR::Topic::FindOrCreate<TsPing>("ts_ping");
    ts_pong_topic_ = LibXR::Topic::FindOrCreate<TsPong>("ts_pong");
    ahrs_stamped_topic_ = LibXR::Topic::FindOrCreate<AhrsStamped>("ahrs_stamped");

    static LibXR::RamFS ramfs;
    static LibXR::LinuxUART uart(global_config().vid, global_config().pid, 115200,
                                 LibXR::LinuxUART::Parity::NO_PARITY, 8, 1);

    static LibXR::HardwareContainer peripherals{
        LibXR::Entry<LibXR::RamFS>({ramfs, {"ramfs"}}),
        LibXR::Entry<LibXR::UART>({uart, {"uart_client"}}),
    };

    static LibXR::ApplicationManager appmgr;
    static SharedTopic shared_topic(peripherals, appmgr, "uart_client", 256,
                                    {{"ts_pong"}, {"ahrs_stamped"}});
    static SharedTopicClient shared_topic_client(peripherals, appmgr, "uart_client", 256,
                                                 {{"ts_ping"}});
  }

  void PhaseRtt()
  {
    std::mutex mtx;
    std::condition_variable cv;
    std::atomic<bool> pong_received{false};
    TsPong last_pong{};

    struct PongCtx
    {
      std::mutex* mtx;
      std::condition_variable* cv;
      std::atomic<bool>* pong_received;
      TsPong* last_pong;
    };

    PongCtx ctx{&mtx, &cv, &pong_received, &last_pong};

    void (*pong_cb_fun)(bool, PongCtx*, LibXR::RawData&) =
        [](bool, PongCtx* c, LibXR::RawData& data)
    {
      auto pong = reinterpret_cast<TsPong*>(data.addr_);
      {
        std::lock_guard<std::mutex> lk(*c->mtx);
        *c->last_pong = *pong;
      }
      c->pong_received->store(true);
      c->cv->notify_one();
    };

    auto pong_cb = LibXR::Topic::Callback::Create(pong_cb_fun, &ctx);
    ts_pong_topic_.RegisterCallback(pong_cb);

    rtt_samples_.clear();
    rtt_samples_.reserve(cfg_.rtt_sample_count);
    clock_samples_.clear();
    clock_samples_.reserve(cfg_.rtt_sample_count);

    int timeout_count = 0;
    for (int i = 0; i < cfg_.rtt_sample_count; i++)
    {
      pong_received.store(false);

      TsPing ping;
      ping.sequence = ping_seq_++;
      double send_time = now_microseconds();
      ts_ping_topic_.Publish(ping);

      {
        std::unique_lock<std::mutex> lk(mtx);
        bool got = cv.wait_for(lk, 500ms, [&]() { return pong_received.load(); });
        if (!got)
        {
          timeout_count++;
          if (timeout_count > 10)
          {
            printf(
                "[WARN] Too many RTT timeouts (%d), stopping RTT "
                "phase\n",
                timeout_count);
            break;
          }
          continue;
        }
      }

      double recv_time = now_microseconds();
      double rtt = recv_time - send_time;

      RttSample sample;
      sample.sequence = last_pong.sequence;
      sample.send_time_us = send_time;
      sample.recv_time_us = recv_time;
      sample.rtt_us = rtt;
      rtt_samples_.push_back(sample);

      clock_samples_.push_back({send_time, rtt, last_pong.mcu_timestamp});

      std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.rtt_interval_ms));
    }

    ASSERT_FALSE(rtt_samples_.empty()) << "No RTT samples collected!";

    std::vector<double> rtt_vals;
    rtt_vals.reserve(rtt_samples_.size());
    for (auto& s : rtt_samples_)
    {
      rtt_vals.push_back(s.rtt_us);
    }
    std::sort(rtt_vals.begin(), rtt_vals.end());
    double median_rtt = rtt_vals[rtt_vals.size() / 2];
    estimated_one_way_delay_us_ = median_rtt / 2.0;

    // C = mcu_ts - (pc_send + rtt/2), filtered to samples within 2x
    // median RTT
    std::vector<double> offsets;
    for (auto& cs : clock_samples_)
    {
      if (cs.rtt < median_rtt * 2.0)
      {
        double offset = static_cast<double>(cs.mcu_ts) - (cs.pc_send + cs.rtt / 2.0);
        offsets.push_back(offset);
      }
    }
    if (!offsets.empty())
    {
      std::sort(offsets.begin(), offsets.end());
      estimated_clock_offset_us_ = offsets[offsets.size() / 2];
    }
    else
    {
      estimated_clock_offset_us_ = 0.0;
    }

    printf(
        "[INFO] RTT: %zu samples, median=%.1f us, one-way=%.1f us, "
        "clock_offset=%.1f us\n",
        rtt_samples_.size(), median_rtt, estimated_one_way_delay_us_,
        estimated_clock_offset_us_);
  }

  void PhaseDelay()
  {
    std::mutex mtx;
    std::condition_variable cv;
    std::atomic<int> count{0};

    delay_samples_.clear();
    delay_samples_.reserve(cfg_.delay_sample_count);

    struct AhrsCtx
    {
      TimestampCalibratorTest* self;
      std::mutex* mtx;
      std::condition_variable* cv;
      std::atomic<int>* count;
    };

    AhrsCtx ctx{this, &mtx, &cv, &count};

    void (*ahrs_cb_fun)(bool, AhrsCtx*, LibXR::RawData&) =
        [](bool, AhrsCtx* c, LibXR::RawData& data)
    {
      auto stamped = reinterpret_cast<AhrsStamped*>(data.addr_);
      double recv_time = now_microseconds();
      double delay = recv_time - static_cast<double>(stamped->mcu_timestamp) +
                     c->self->estimated_clock_offset_us_;

      {
        std::lock_guard<std::mutex> lk(*c->mtx);
        DelaySample ds;
        ds.pc_recv_time_us = recv_time;
        ds.mcu_timestamp = stamped->mcu_timestamp;
        ds.estimated_delay_us = delay;
        c->self->delay_samples_.push_back(ds);
      }
      c->count->fetch_add(1);
      c->cv->notify_one();
    };

    auto ahrs_cb = LibXR::Topic::Callback::Create(ahrs_cb_fun, &ctx);
    ahrs_stamped_topic_.RegisterCallback(ahrs_cb);

    {
      std::unique_lock<std::mutex> lk(mtx);
      cv.wait_for(lk, std::chrono::seconds(cfg_.delay_sample_timeout_s),
                  [&]() { return count.load() >= cfg_.delay_sample_count; });
    }

    printf("[INFO] Delay phase: collected %d samples\n", count.load());
  }

  void PrintResults()
  {
    printf("============================================\n");
    printf("    Timestamp Offset Calibration Results\n");
    printf("============================================\n");

    if (!rtt_samples_.empty())
    {
      std::vector<double> rtt_vals;
      rtt_vals.reserve(rtt_samples_.size());
      for (auto& s : rtt_samples_)
      {
        rtt_vals.push_back(s.rtt_us);
      }
      std::sort(rtt_vals.begin(), rtt_vals.end());
      double sum = std::accumulate(rtt_vals.begin(), rtt_vals.end(), 0.0);
      double mean = sum / static_cast<double>(rtt_vals.size());
      double median = rtt_vals[rtt_vals.size() / 2];
      double sq_sum = 0.0;
      for (auto v : rtt_vals)
      {
        sq_sum += (v - mean) * (v - mean);
      }
      double stddev = std::sqrt(sq_sum / static_cast<double>(rtt_vals.size()));
      double p5 =
          rtt_vals[static_cast<size_t>(static_cast<double>(rtt_vals.size()) * 0.05)];
      double p95 =
          rtt_vals[static_cast<size_t>(static_cast<double>(rtt_vals.size()) * 0.95)];

      printf("[RTT] samples=%zu\n", rtt_vals.size());
      printf("[RTT] mean=%.1f us, median=%.1f us, stddev=%.1f us\n", mean, median,
             stddev);
      printf("[RTT] min=%.1f us, max=%.1f us\n", rtt_vals.front(), rtt_vals.back());
      printf("[RTT] p5=%.1f us, p95=%.1f us\n", p5, p95);
      printf("[RTT] estimated one-way delay=%.1f us\n", estimated_one_way_delay_us_);
    }

    if (!delay_samples_.empty())
    {
      std::vector<double> delay_vals;
      delay_vals.reserve(delay_samples_.size());
      for (auto& s : delay_samples_)
      {
        delay_vals.push_back(s.estimated_delay_us);
      }
      std::sort(delay_vals.begin(), delay_vals.end());
      double sum = std::accumulate(delay_vals.begin(), delay_vals.end(), 0.0);
      double mean = sum / static_cast<double>(delay_vals.size());
      double median = delay_vals[delay_vals.size() / 2];
      double sq_sum = 0.0;
      for (auto v : delay_vals)
      {
        sq_sum += (v - mean) * (v - mean);
      }
      double stddev = std::sqrt(sq_sum / static_cast<double>(delay_vals.size()));
      double p5 =
          delay_vals[static_cast<size_t>(static_cast<double>(delay_vals.size()) * 0.05)];
      double p95 =
          delay_vals[static_cast<size_t>(static_cast<double>(delay_vals.size()) * 0.95)];

      printf("[Delay] samples=%zu\n", delay_vals.size());
      printf("[Delay] mean=%.1f us, median=%.1f us, stddev=%.1f us\n", mean, median,
             stddev);
      printf("[Delay] min=%.1f us, max=%.1f us\n", delay_vals.front(), delay_vals.back());
      printf("[Delay] p5=%.1f us, p95=%.1f us\n", p5, p95);

      double recommended_offset_s = -(median / 1e6);
      printf("--------------------------------------------\n");
      printf("Recommended timestamp_offset: %.6f s\n", recommended_offset_s);
      printf("--------------------------------------------\n");
    }
    else
    {
      printf(
          "[WARN] No delay samples collected, cannot recommend "
          "offset\n");
    }

    printf("Calibration complete.\n");
  }

  TestConfig cfg_;
  uint32_t ping_seq_{0};

  std::vector<RttSample> rtt_samples_;
  std::vector<ClockSample> clock_samples_;
  double estimated_one_way_delay_us_{0.0};
  double estimated_clock_offset_us_{0.0};
  std::vector<DelaySample> delay_samples_;

  LibXR::Topic ts_ping_topic_;
  LibXR::Topic ts_pong_topic_;
  LibXR::Topic ahrs_stamped_topic_;
};

TEST_F(TimestampCalibratorTest, RunCalibration)
{
  printf(
      "[INFO] TimestampCalibrator initialized, starting "
      "calibration...\n");

  // 等待串口连接稳定
  std::this_thread::sleep_for(2s);

  printf("[INFO] === Phase 1: RTT Measurement ===\n");
  ASSERT_NO_FATAL_FAILURE(PhaseRtt());

  printf("[INFO] === Phase 2: One-way Delay Measurement ===\n");
  PhaseDelay();

  ASSERT_FALSE(delay_samples_.empty())
      << "No delay samples collected, cannot recommend offset";

  PrintResults();
}

}  // namespace

int main(int argc, char** argv)
{
  parse_custom_args(&argc, argv, global_config());
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

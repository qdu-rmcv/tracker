#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <queue>
#include <string>
#include <thread>
#include <vector>

// 表参数配置
constexpr double MIN_PITCH = -0.726893;  // 最小pitch限位 (rad)
constexpr double MAX_PITCH = 0.885398;   // 最大pitch限位 (rad)
constexpr double MAX_X = 22.0;           // 最大水平距离 (m)
constexpr double MIN_X = -5.0;            // 最小水平距离 (m)
constexpr double MAX_Y = 3.0;            // 最大高度 (m)
constexpr double MIN_Y = -4.0;           // 最小高度 (m)
constexpr double RESOLUTION = 0.01;      // 精度 (m)
constexpr double MAX_ERROR = 0.01;       // 允许误差 (m)
constexpr int ERROR_LEVEL = 10;          // 误差等级
constexpr double GUN = 0.15;             // 枪口到pitch轴电机的距离 (m)

constexpr double G = 9.7985;     // 重力加速度 (m/s^2)
constexpr double STEP = 0.0001;  // RK4步长 (s)
static double v0 = 22.7;         // 默认弹速
static bool bullet_type = 1;     // 默认42mm (英雄)

// 弹丸状态
struct State
{
  double x, y, vx, vy;
  State(double x, double y, double vx, double vy) : x(x), y(y), vx(vx), vy(vy) {}
};

using Clock = std::chrono::steady_clock;
using Seconds = std::chrono::duration<double>;

// 性能统计
struct PerfStats
{
  Clock::time_point program_start;
  Clock::time_point compute_start;
  Clock::time_point compute_end;
  Clock::time_point write_start;
  Clock::time_point write_end;

  std::atomic<size_t> total_cells{0};
  std::atomic<size_t> solved_cells{0};
  std::atomic<size_t> failed_cells{0};
  std::atomic<size_t> rows_done{0};

  void MarkProgramStart() { program_start = Clock::now(); }
  void MarkComputeStart() { compute_start = Clock::now(); }
  void MarkComputeEnd() { compute_end = Clock::now(); }
  void MarkWriteStart() { write_start = Clock::now(); }
  void MarkWriteEnd() { write_end = Clock::now(); }

  static double SecBetween(const Clock::time_point& a, const Clock::time_point& b)
  {
    return std::chrono::duration_cast<Seconds>(b - a).count();
  }

  void PrintSummary(size_t total_rows, size_t y_dim, double init_v, bool type) const
  {
    const double COMPUTE_SEC = SecBetween(compute_start, compute_end);
    const double WRITE_SEC = SecBetween(write_start, write_end);
    const double TOTAL_SEC = SecBetween(program_start, write_end);

    const size_t TOTAL = total_cells.load();
    const size_t SOLVED = solved_cells.load();
    const size_t FAILED = failed_cells.load();
    const size_t ROWS = rows_done.load();

    const double HIT_RATE =
        (TOTAL > 0) ? (100.0 * static_cast<double>(SOLVED) / TOTAL) : 0.0;
    const double ROWS_PER_SEC =
        (COMPUTE_SEC > 0.0) ? (static_cast<double>(ROWS) / COMPUTE_SEC) : 0.0;
    const double CELLS_PER_SEC =
        (COMPUTE_SEC > 0.0) ? (static_cast<double>(TOTAL) / COMPUTE_SEC) : 0.0;
    const double AVG_SEC_PER_ROW =
        (ROWS > 0) ? (COMPUTE_SEC / static_cast<double>(ROWS)) : 0.0;
    const double AVG_US_PER_CELL =
        (TOTAL > 0) ? (COMPUTE_SEC * 1e6 / static_cast<double>(TOTAL)) : 0.0;

    std::cerr << "\n========== 性能统计汇总 ==========\n";
    std::cerr << "弹速: " << init_v << " m/s\n";
    std::cerr << "弹丸类型: " << (type ? "17mm" : "42mm") << "\n";
    std::cerr << "表尺寸: " << total_rows << " x " << y_dim << " = "
              << static_cast<uint64_t>(total_rows) * y_dim << " cells\n";
    std::cerr << "有效解数量: " << SOLVED << "\n";
    std::cerr << "无解数量: " << FAILED << "\n";
    std::cerr << "命中率: " << std::fixed << std::setprecision(2) << HIT_RATE << " %\n";
    std::cerr << "计算阶段耗时: " << COMPUTE_SEC << " s\n";
    std::cerr << "写文件阶段耗时: " << WRITE_SEC << " s\n";
    std::cerr << "总耗时: " << TOTAL_SEC << " s\n";
    std::cerr << "平均每行耗时: " << AVG_SEC_PER_ROW << " s/row\n";
    std::cerr << "平均每单元耗时: " << AVG_US_PER_CELL << " us/cell\n";
    std::cerr << "平均行吞吐: " << ROWS_PER_SEC << " rows/s\n";
    std::cerr << "平均单元吞吐: " << CELLS_PER_SEC << " cells/s\n";
    std::cerr << "==================================\n";
  }
};

// 弹道解算器
class TrajectoryTableGenerator
{
 private:
  double v0_;        // 初速度
  double k_;         // 阻力系数
  double target_x_;  // 目标x坐标
  double target_y_;  // 目标y坐标
  double dt_;        // RK4步长

 public:
  TrajectoryTableGenerator(double v0, bool type, double target_x, double target_y,
                           double dt = STEP)
      : v0_(v0), target_x_(target_x), target_y_(target_y), dt_(dt)
  {
    if (type == 0)
    {
      // 英雄 (42mm弹丸)
      // 20度时空气密度、空气阻力系数、子弹直径、重量
      k_ = 1.205 * 0.40 * 0.0425 * 0.0425 / (2 * 0.0445);
    }
    else
    {
      // 步兵 (17mm弹丸)
      k_ = 1.205 * 0.47 * 0.0168 * 0.0168 / (2 * 0.0032);
    }
  }

  // 运动方程: dy/dt = f(t, y)
  std::vector<double> AirODE(const State& state)
  {
    double v = std::sqrt(state.vx * state.vx + state.vy * state.vy);
    double ax = -k_ * v * state.vx;
    double ay = -G - k_ * v * state.vy;
    return {state.vx, state.vy, ax, ay};
  }

  // RK4 单步积分
  State RK4Step(const State& state, double h)
  {
    auto k1 = AirODE(state);
    State state1(state.x + 0.5 * h * k1[0], state.y + 0.5 * h * k1[1],
                 state.vx + 0.5 * h * k1[2], state.vy + 0.5 * h * k1[3]);

    auto k2 = AirODE(state1);
    State state2(state.x + 0.5 * h * k2[0], state.y + 0.5 * h * k2[1],
                 state.vx + 0.5 * h * k2[2], state.vy + 0.5 * h * k2[3]);

    auto k3 = AirODE(state2);
    State state3(state.x + h * k3[0], state.y + h * k3[1], state.vx + h * k3[2],
                 state.vy + h * k3[3]);

    auto k4 = AirODE(state3);

    State new_state(state.x, state.y, state.vx, state.vy);
    new_state.x += h * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]) / 6.0;
    new_state.y += h * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]) / 6.0;
    new_state.vx += h * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]) / 6.0;
    new_state.vy += h * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]) / 6.0;

    return new_state;
  }

  // 二分法搜索pitch
  std::vector<double> SolvePitch(double error)
  {
    double t_b = GUN / v0_;
    double pitch_top = MAX_PITCH;
    double pitch_low = MIN_PITCH;

    while ((pitch_top - pitch_low) > 0.001)
    {
      double count = 0;
      double pitch_binary = (pitch_top + pitch_low) / 2.0;
      double x_b = -GUN * std::cos(pitch_binary);
      double y_b = -GUN * std::sin(pitch_binary);
      double x_to_gun = target_x_ + x_b;
      double y_to_gun = target_y_ + y_b;
      State state(0, 0, v0_ * std::cos(pitch_binary), v0_ * std::sin(pitch_binary));

      while (state.y >= MIN_Y - 1.0)
      {
        state = RK4Step(state, dt_);
        count++;

        if (std::pow(state.x - x_to_gun, 2) + std::pow(state.y - y_to_gun, 2) <=
            std::pow(error, 2))
        {
          return {pitch_binary, count * dt_ + t_b,
                  std::sqrt(state.vx * state.vx + state.vy * state.vy)};
        }

        if (state.x >= x_to_gun)
        {
          if (state.y > y_to_gun)
          {
            pitch_top = pitch_binary;
          }
          else
          {
            pitch_low = pitch_binary;
          }
          break;
        }
        else if (state.y < MIN_Y - 1.0 && state.x < x_to_gun)
        {
          pitch_low = pitch_binary;
          break;
        }
      }
    }
    return {NAN, NAN, NAN};
  }

  // 多级误差搜索
  std::vector<double> SolvePitchLevel(int error_level)
  {
    auto ge = SolvePitch(MAX_ERROR);
    if (std::isnan(ge[0]))
    {
      return {NAN, NAN, NAN};
    }

    for (int i = 1; i <= error_level; i++)
    {
      ge = SolvePitch(MAX_ERROR / error_level * i);
      if (!std::isnan(ge[0]))
      {
        return ge;
      }
    }
    return {NAN, NAN, NAN};
  }
};

using TableData = std::vector<std::vector<std::vector<double>>>;

// 单任务求解结果
struct SolveResult
{
  TableData table;
  size_t solved_cells = 0;
  size_t failed_cells = 0;
  double elapsed_sec = 0.0;
};

// 解算单行
static SolveResult solve_rows(double start_x, size_t num_rows, double v0,
                              bool bullet_type)
{
  auto task_start = Clock::now();

  double x = start_x;
  size_t y_dim = static_cast<size_t>(std::round((MAX_Y - MIN_Y) / RESOLUTION + 1));
  SolveResult result;
  result.table.reserve(num_rows);

  for (size_t i = 0; i < num_rows; i++, x += RESOLUTION)
  {
    double y = MIN_Y;
    std::vector<std::vector<double>> row;
    row.reserve(y_dim);

    for (size_t j = 0; j < y_dim; j++, y += RESOLUTION)
    {
      TrajectoryTableGenerator solve(v0, bullet_type, x, y);
      std::vector<double> ge = solve.SolvePitchLevel(ERROR_LEVEL);

      if (!ge.empty() && !std::isnan(ge[0]))
      {
        result.solved_cells++;
      }
      else
      {
        result.failed_cells++;
      }

      row.push_back(std::move(ge));
    }

    result.table.push_back(std::move(row));
  }

  auto task_end = Clock::now();
  result.elapsed_sec = std::chrono::duration_cast<Seconds>(task_end - task_start).count();
  return result;
}

// 输出表格解的情况
template <typename T>
static std::ostream& operator<<=(std::ostream& os, const std::vector<T>& v)
{
  for (const T& x : v)
  {
    os <<= x;
  }
  os << "\n";
  return os;
}

template <>
std::ostream& operator<<=(std::ostream& os, const std::vector<double>& v)
{
  return os << (std::isnan(v[0]) ? ' ' : '.');
}

static void build_table(double init_v0, bool type, const std::string& output_prefix)
{
  PerfStats perf;
  perf.MarkProgramStart();

  TableData table;
  std::ios_base::sync_with_stdio(false);
  std::queue<std::future<SolveResult>> futures;

  size_t threads = std::thread::hardware_concurrency();
  if (threads == 0)
  {
    threads = 16;
  }

  const size_t TOTAL_ROWS =
      static_cast<size_t>(std::round((MAX_X - MIN_X) / RESOLUTION + 1));
  const size_t Y_DIM = static_cast<size_t>(std::round((MAX_Y - MIN_Y) / RESOLUTION + 1));
  const size_t TOTAL_CELLS = TOTAL_ROWS * Y_DIM;

  perf.total_cells = TOTAL_CELLS;
  table.reserve(TOTAL_ROWS);

  std::cerr << "开始生成弹道查找表..." << '\n';
  std::cerr << "弹速: " << init_v0 << " m/s" << '\n';
  std::cerr << "弹丸类型: " << (type ? "17mm" : "42mm") << '\n';
  std::cerr << "使用 " << threads << " 个线程" << '\n';
  std::cerr << "总行数: " << TOTAL_ROWS << '\n';
  std::cerr << "每行列数: " << Y_DIM << '\n';
  std::cerr << "总单元数: " << TOTAL_CELLS << '\n';

  perf.MarkComputeStart();

  double current_x = MIN_X;
  size_t rows_processed = 0;
  size_t batch_index = 0;

  while (rows_processed < TOTAL_ROWS)
  {
    auto batch_start = Clock::now();
    const size_t BATCH_SIZE = std::min(threads, TOTAL_ROWS - rows_processed);

    // 保持稳定性：一任务一行
    for (size_t i = 0; i < BATCH_SIZE; ++i)
    {
      futures.push(
          std::async(std::launch::async, solve_rows, current_x, 1, init_v0, type));
      current_x += RESOLUTION;
    }

    double batch_task_sum_sec = 0.0;
    size_t batch_solved = 0;
    size_t batch_failed = 0;

    while (!futures.empty())
    {
      SolveResult result = futures.front().get();
      futures.pop();

      batch_task_sum_sec += result.elapsed_sec;
      batch_solved += result.solved_cells;
      batch_failed += result.failed_cells;

      table.insert(table.end(), std::make_move_iterator(result.table.begin()),
                   std::make_move_iterator(result.table.end()));
    }

    rows_processed += BATCH_SIZE;
    perf.rows_done = rows_processed;
    perf.solved_cells += batch_solved;
    perf.failed_cells += batch_failed;

    auto batch_end = Clock::now();
    const double BATCH_WALL_SEC =
        std::chrono::duration_cast<Seconds>(batch_end - batch_start).count();

    const double PROGRESS =
        100.0 * static_cast<double>(rows_processed) / static_cast<double>(TOTAL_ROWS);
    const double AVG_ROW_WALL =
        (BATCH_SIZE > 0) ? (BATCH_WALL_SEC / static_cast<double>(BATCH_SIZE)) : 0.0;
    const double BATCH_ROWS_PER_SEC =
        (BATCH_WALL_SEC > 0.0) ? (static_cast<double>(BATCH_SIZE) / BATCH_WALL_SEC) : 0.0;
    const double BATCH_HIT_RATE = ((batch_solved + batch_failed) > 0)
                                      ? (100.0 * static_cast<double>(batch_solved) /
                                         static_cast<double>(batch_solved + batch_failed))
                                      : 0.0;

    std::cerr << std::fixed << std::setprecision(2);
    std::cerr << "[Batch " << (++batch_index) << "] "
              << "进度: " << rows_processed << " / " << TOTAL_ROWS << " (" << PROGRESS
              << "%), "
              << "批次耗时: " << BATCH_WALL_SEC << " s, "
              << "平均每行墙钟耗时: " << AVG_ROW_WALL << " s, "
              << "批次吞吐: " << BATCH_ROWS_PER_SEC << " rows/s, "
              << "任务累计耗时: " << batch_task_sum_sec << " s, "
              << "有效解: " << batch_solved << ", "
              << "无解: " << batch_failed << ", "
              << "批次命中率: " << BATCH_HIT_RATE << " %" << '\n';
  }

  perf.MarkComputeEnd();

  std::cerr << "计算完成。" << '\n';
  (std::cerr <<= table) << '\n';

  perf.MarkWriteStart();

  // 二进制文件写入，保持 double 精度
  std::string output_filename =
      output_prefix + "_" + std::to_string(static_cast<int>(MAX_X)) + "_table.bin";
  std::ofstream file_out(output_filename.c_str(), std::ios::out | std::ios::binary);
  if (!file_out)
  {
    std::cerr << "错误: 无法打开文件进行写入: " << output_filename << '\n';
    return;
  }

  struct Cell
  {
    double pitch;
    double t;
    double v;
  };

  for (const auto& row : table)
  {
    for (const auto& ge : row)
    {
      Cell cell_to_write;
      if (ge.empty() || std::isnan(ge[0]))
      {
        cell_to_write = {NAN, NAN, NAN};
      }
      else
      {
        cell_to_write = {ge[0], ge[1], ge[2]};
      }
      file_out.write(reinterpret_cast<const char*>(&cell_to_write), sizeof(Cell));
    }
  }

  file_out.close();
  perf.MarkWriteEnd();

  std::cerr << "二进制查找表已成功生成到 " << output_filename << '\n';
  perf.PrintSummary(TOTAL_ROWS, Y_DIM, init_v0, type);
}

int main(int argc, char* argv[])
{
  std::string prefix = bullet_type ? "infantry" : "hero";

  if (argc >= 2)
  {
    v0 = std::stod(argv[1]);
  }
  if (argc >= 3)
  {
    bullet_type = std::stoi(argv[2]);
    prefix = bullet_type ? "infantry" : "hero";
  }
  if (argc >= 4)
  {
    prefix = argv[3];
  }

  build_table(v0, bullet_type, prefix);
  return 0;
}

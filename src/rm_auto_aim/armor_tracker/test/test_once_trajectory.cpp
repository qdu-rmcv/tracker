#include <cmath>
#include <iostream>
#include <vector>

constexpr double MIN_PITCH = -0.226893;  // 限位
constexpr double MAX_PITCH = 0.785398;
constexpr double MIN_Y = -1;         // 中心为车的pitch轴电机
constexpr double MAX_ERROR = 0.005;  // 允许误差，m
constexpr int ERROR_LEVEL = 5;       // 误差等级
constexpr double GUN = 0.15;         // 枪口到pitch轴电机的距离，m

constexpr double G = 9.8;        // 重力加速度，m/s^2
constexpr double STEP = 0.0001;  // RK4步长 (s)

constexpr double BULLET_V = 11.5;

struct State
{
  double x, y, vx, vy;
  State(double x, double y, double vx, double vy) : x(x), y(y), vx(vx), vy(vy) {}
};

class SolveTrajectory
{
 private:
  double v0_;
  double k_;         // 阻力系数
  double target_x_;  // 目标x，y坐标 ,相对小车pitch轴电机(小车中心点)
  double target_y_;
  double dt_;  // RK4步长 (s)

 public:
  SolveTrajectory(double v0, bool type, double target_x, double target_y,
                  double dt = STEP)
      : v0_(v0), target_x_(target_x), target_y_(target_y), dt_(dt)
  {
    if (type == 0)
    {  // 英雄
      // 20度时空气密度、空气阻力系数、子弹直径、重量
      k_ = 1.205 * 0.40 * 0.0425 * 0.0425 / (2 * 0.0445);
    }
    else if (type == 1)
    {
      k_ = 1.205 * 0.47 * 0.0168 * 0.0168 / (2 * 0.0032);
    }
  }

  // 运动方程: dy/dt = f(t, y)
  std::vector<double> AirODE(const State& state)
  {
    double v = std::sqrt(state.vx * state.vx + state.vy * state.vy);
    double ax = -k_ * v * state.vx;       // dvx/dt
    double ay = -G - k_ * v * state.vy;   // dvy/dt
    return {state.vx, state.vy, ax, ay};  // [dx/dt, dy/dt, dvx/dt, dvy/dt]
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

  // 重载SolvePitch，当不提供pitch时默认使用二分法搜索
  std::vector<double> SolvePitch(double error)
  {
    double t_b = GUN / v0_;
    double pitch_top = MAX_PITCH;
    double pitch_low = MIN_PITCH;
    while ((pitch_top - pitch_low) > 0.001)
    {
      double count = 0;
      double pitch_binary = (pitch_top + pitch_low) / 2;
      double x_b = -GUN * std::cos(pitch_binary);
      double y_b = -GUN * std::sin(pitch_binary);
      double x_to_gun = target_x_ + x_b;
      double y_to_gun = target_y_ + y_b;
      State state(0, 0, v0_ * std::cos(pitch_binary), v0_ * std::sin(pitch_binary));

      // 这里用MIN_Y-1就能运行，MIN_Y就不行，不要问为什么
      while (state.y >= MIN_Y - 1)
      {
        state = RK4Step(state, dt_);
        count++;
        if (pow(state.x - x_to_gun, 2) + pow(state.y - y_to_gun, 2) <= pow(error, 2))
        {
          return {pitch_binary, count * dt_ + t_b,
                  std::sqrt(state.vx * state.vx + state.vy * state.vy)};
        }

        if (state.x >= x_to_gun)
        {
          if (state.y > y_to_gun)
            pitch_top = pitch_binary;
          else
            pitch_low = pitch_binary;
          break;
        }
        else if (state.y < MIN_Y - 1 && state.x < x_to_gun)
        {
          pitch_low = pitch_binary;
          break;
        }
      }
    }
    return {NAN, NAN, NAN};
  }

  // 对solvePitch的优化，考虑多级误差以保证精确和有解
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

  // 给角度解位置
  std::vector<double> SolveHeightAndLength(double pitch)
  {
    double max_heigh = 0;
    double max_length = 0;
    State state(0, 0, v0_ * std::cos(pitch), v0_ * std::sin(pitch));
    while (state.y > -0.5)
    {
      state = RK4Step(state, dt_);
      if (max_heigh < state.y)
      {
        max_heigh = state.y;
      }
      if (max_length < state.x)
      {
        max_length = state.x;
      }
    }
    return {max_heigh, max_length};
  }
};

int main(int argc, char** argv)
{
  std::vector<double> ans;
  if (argc == 2)
  {
    double pitch = atof(argv[1]);
    SolveTrajectory solve = SolveTrajectory(BULLET_V, 0, {}, {});
    ans = solve.SolveHeightAndLength(pitch);

    std::cout << "Max Height: " << ans[0] << " Max Length: " << ans[1] << '\n';
  }
  else if (argc == 3)
  {
    double distance = atof(argv[1]);
    double height = atof(argv[2]);
    SolveTrajectory solve = SolveTrajectory(BULLET_V, 0, distance, height);
    ans = solve.SolvePitchLevel(ERROR_LEVEL);

    std::cout << "Pitch: " << ans[0] << " Time: " << ans[1] << " Velocity: " << ans[2]
              << '\n';
  }
  else
  {
    std::cout << "Usage: trajectory_test <pitch>(Rad)\n"
              << "       trajectory_test <distance>(m) <height>(m)\n";
    return 0;
  }

  return 0;
}
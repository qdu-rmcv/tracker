#pragma once

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <vector>

class Table
{
 public:
  struct TableConfig
  {
    double max_x, min_x, max_y, min_y, resolution;
    size_t x_dim, y_dim;
    std::string filename;
    TableConfig() = default;
    TableConfig(const TableConfig&) = default;
    TableConfig(double max_x, double min_x, double max_y, double min_y, double resolution,
                std::string filename)
        : max_x(max_x),
          min_x(min_x),
          max_y(max_y),
          min_y(min_y),
          resolution(resolution),
          x_dim(static_cast<size_t>((max_x - min_x) / resolution) + 1),
          y_dim(static_cast<size_t>((max_y - min_y) / resolution) + 1),
          filename(std::move(filename))
    {
    }
  };

  struct Cell
  {
    double pitch;
    double t;
    double v;
  };

  explicit Table(const TableConfig& config)
      : MAX_X(config.max_x),
        MIN_X(config.min_x),
        MAX_Y(config.max_y),
        MIN_Y(config.min_y),
        RESOLUTION(config.resolution),
        X_DIM(config.x_dim),
        Y_DIM(config.y_dim),
        filename_(config.filename)
  {
  }

  ~Table() = default;

  // 查表获取弹道参数
  Cell Check(double x, double y) const
  {
    if (!init_)
    {
      return {NAN, NAN, NAN};
    }

    // 边界检查
    double adjusted_x = x;
    double adjusted_y = y;

    if (adjusted_x < MIN_X || adjusted_x > MAX_X || adjusted_y < MIN_Y ||
        adjusted_y > MAX_Y)
    {
      return {NAN, NAN, NAN};
    }

    // 将坐标映射到网格坐标系
    const double fx = (x - MIN_X) / RESOLUTION;
    const double fy = (y - MIN_Y) / RESOLUTION;

    // 左下角索引
    size_t x0 = static_cast<size_t>(std::floor(fx));
    size_t y0 = static_cast<size_t>(std::floor(fy));

    // 处理边界：如果刚好落在最大边界，避免 x1/y1 越界
    size_t x1 = std::min(x0 + 1, X_DIM - 1);
    size_t y1 = std::min(y0 + 1, Y_DIM - 1);

    x0 = std::min(x0, X_DIM - 1);
    y0 = std::min(y0, Y_DIM - 1);

    // 小数部分，作为插值权重
    const double dx = fx - static_cast<double>(x0);
    const double dy = fy - static_cast<double>(y0);

    const Cell& c00 = table_[x0 * Y_DIM + y0];
    const Cell& c10 = table_[x1 * Y_DIM + y0];
    const Cell& c01 = table_[x0 * Y_DIM + y1];
    const Cell& c11 = table_[x1 * Y_DIM + y1];

    auto bilerp = [dx, dy](double v00, double v10, double v01, double v11) -> double
    {
      return (1.0 - dx) * (1.0 - dy) * v00 + dx * (1.0 - dy) * v10 +
             (1.0 - dx) * dy * v01 + dx * dy * v11;
    };

    return {bilerp(c00.pitch, c10.pitch, c01.pitch, c11.pitch),
            bilerp(c00.t, c10.t, c01.t, c11.t), bilerp(c00.v, c10.v, c01.v, c11.v)};
  }

  // 初始化：从二进制文件加载表
  bool Init()
  {
    table_.resize(X_DIM * Y_DIM);

    std::ifstream file_in(filename_, std::ios::binary);
    if (!file_in.is_open())
    {
      std::cerr << "[TrajectoryTable] 无法打开文件: " << filename_ << '\n';
      init_ = false;
      return false;
    }

    file_in.seekg(0, std::ios::end);
    const std::streamsize file_size = file_in.tellg();
    file_in.seekg(0, std::ios::beg);

    const std::streamsize expected_size =
        static_cast<std::streamsize>(X_DIM * Y_DIM * sizeof(Cell));

    std::cerr << "[TrajectoryTable] file: " << filename_ << '\n';
    std::cerr << "[TrajectoryTable] X_DIM=" << X_DIM << " Y_DIM=" << Y_DIM
              << " sizeof(Cell)=" << sizeof(Cell) << '\n';
    std::cerr << "[TrajectoryTable] expected=" << expected_size << " actual=" << file_size
              << '\n';

    if (file_size != expected_size)
    {
      std::cerr << "[TrajectoryTable] 文件大小不匹配，拒绝加载\n";
      init_ = false;
      return false;
    }

    file_in.read(reinterpret_cast<char*>(table_.data()), expected_size);

    if (file_in.bad())
    {
      std::cerr << "[TrajectoryTable] 底层IO错误\n";
      init_ = false;
      return false;
    }

    if (file_in.gcount() != expected_size)
    {
      std::cerr << "[TrajectoryTable] 读取字节数不匹配，期望: " << expected_size
                << " 实际: " << file_in.gcount() << '\n';
      init_ = false;
      return false;
    }

    file_in.close();
    init_ = true;
    std::cout << "[TrajectoryTable] 弹道查找表加载成功: " << filename_ << '\n';
    return true;
  }

  bool IsInit() const { return init_; }

 private:
  double MAX_X;
  double MIN_X;
  double MAX_Y;
  double MIN_Y;
  double RESOLUTION;

  size_t X_DIM;
  size_t Y_DIM;

  bool init_ = false;
  std::string filename_;
  std::vector<Cell> table_;
};
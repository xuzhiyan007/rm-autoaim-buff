#include "auto_aim/target.hpp"

#include <numeric>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/ros_time.hpp"

namespace auto_aim
{
Target::Target(
  const Armor_point & armor, rclcpp::Time t, double radius, int armor_num,
  Eigen::VectorXd P0_dig)
: name(armor.name),
  armor_type(armor.type),
  jumped(false),
  last_id(0),
  update_count_(0),
  armor_num_(armor_num),
  t_(t),
  is_switch_(false),
  is_converged_(false),
  switch_count_(0),
  rebuilt_(false)
{
  auto r = radius;
  const Eigen::VectorXd & xyz = armor.xyz_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;

  // 旋转中心的坐标
  auto center_x = xyz[0] + r * std::cos(ypr[0]);
  auto center_y = xyz[1] + r * std::sin(ypr[0]);
  auto center_z = xyz[2];

  // x vx y vy z vz a w r l h
  // a: angle
  // w: angular velocity
  // l: r2 - r1
  // h: z2 - z1
  Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0}};  //初始化预测量
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
}

Target::Target(double x, double vyaw, double radius, double h) : armor_num_(4)
{
  Eigen::VectorXd x0{{x, 0, 0, 0, 0, 0, 0, vyaw, radius, 0, h}};
  Eigen::VectorXd P0_dig{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
}

void Target::predict(rclcpp::Time t)
{
  auto dt = tools::delta_time(t, t_);
  std::cout<<"dt  "<<dt<<"\n";
  predict(dt);
  t_ = t;
}

void Target::predict(double dt)
{
  // 状态转移矩阵
  // clang-format off

  // x  vx   y   vy  z   vz  a   w   r   l   h
  Eigen::MatrixXd F{
    {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
  };
  // clang-format on

  // Piecewise White Noise Model
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  double v1, v2 ,q_r;
  if (name == ArmorName::outpost) {
    v1 = 0;   // 前哨站加速度方差 10
    v2 = 0.1;  // 前哨站角加速度方差
  } else {
    v1 = 100;  // 加速度方差 sigma2_q_xyz
    v2 = 300;  // 角加速度方差 sigma2_q_yaw
  }
  q_r = 1e-5;
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  // 预测过程噪声偏差的方差
  // clang-format off
    //    x      vx       y       vy      z       vz      a       w  r  l  h
  Eigen::MatrixXd Q{
    {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, a * v1 , b * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, b * v1 , c * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, q_r, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, q_r, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, q_r}
  };
  // clang-format on

  // 防止夹角求和出现异常值
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);
    return x_prior;
  };

  // 前哨站转速特判
  if (this->convergened() && this->name == ArmorName::outpost && std::abs(this->ekf_.x[7]) > 2) {
    this->ekf_.x[7] = this->ekf_.x[7] > 0 ? 2.51 : -2.51;
    if(jumped && abs(this->ekf_.x[10])>0.06)
    {
      auto sign_ = this->ekf_.x[10] >0 ? 1 :-1; 
      this->ekf_.x[10] = abs(this->ekf_.x[10]-0.1)<abs(this->ekf_.x[10]-0.2) ? 0.1 : 0.2;
      this->ekf_.x[10] *= sign_;  
    }
  //   this->ekf_.x[7] = this->ekf_.x[7] > 0 ? 1.7 : -1.7;
  }
  // this->ekf_.x[4] = -0.1;
  // this->ekf_.x[5] = 0;
  ekf_.predict(F, Q, f);
}

void Target::update(const Armor_point & armor)
{
  // 装甲板匹配
  int id = 0;
  auto min_angle_error = 1e10;
  const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();

  std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
  for (int i = 0; i < armor_num_; i++) {
    xyza_i_list.push_back({xyza_list[i], i});
  }

  std::sort(
    xyza_i_list.begin(), xyza_i_list.end(),
    [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
      Eigen::Vector3d ypd1 = tools::xyz2ypd(a.first.head(3));
      Eigen::Vector3d ypd2 = tools::xyz2ypd(b.first.head(3));
      return ypd1[2] < ypd2[2];
    });

  // 取前3个distance最小的装甲板
  for (int i = 0; i < 3; i++) {
    const auto & xyza = xyza_i_list[i].first;
    Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
    // auto angle_error = std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));
    auto angle_error =  std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0])) + std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3]));

    if (std::abs(angle_error) < std::abs(min_angle_error)) {
      id = xyza_i_list[i].second;
      min_angle_error = angle_error;
    }
  }

  // if(min_angle_error>0.6 and found_==1)
  // {

  // }

  //id：筛选出和被观测的armor的yaw_error最小的状态对应id
  if (id != 0) jumped = true;  //jumped：是否观测到第二块板
  if(id<4 && id>=0)v_id[id] = 1;
  if (armor_num_ == 3 && id >= 0 && id < 3) {
    z_at_id_[id] = armor.xyz_in_world[2];
  }
  now_id = id;
  if (id != last_id) {
    is_switch_ = true;
  } else {
    is_switch_ = false;
  }

  if (is_switch_) switch_count_++;

  update_count_++;

  if(!rebuilt_ && check_rebuild_optpost())
  {
    rebuild_outpost(armor.xyz_in_world[2],id);
  }

  last_id = id;

  std::cout<<"!!!id: "<<id<<"\n";
  update_ypda(armor, id);
}

bool Target::check_rebuild_optpost()
{
  return armor_num_ == 3 && update_count_>=2 && v_id[0] && v_id[1] && v_id[2];
}

void Target::rebuild_outpost(double z_now,int& id)
{
  // 约定：x[4] = 三板中最高板的 z，x[10] = -0.1m，z(id) = x[4] + x[10]*id
  // 物理 H→M→L 必须沿 id 递增方向（与模型角度公式 angle = x[6] + id*2π/N 同向）
  // 用 z 数据判方向：从 id_high 走 id+1 该是 M（z 比走 id-1 那块大）
  z_at_id_[id] = z_now;

  int id_high = 0;
  for (int i = 1; i < armor_num_; i++) {
    if (z_at_id_[i] > z_at_id_[id_high]) id_high = i;
  }

  double z_next = z_at_id_[(id_high + 1) % armor_num_];
  double z_prev = z_at_id_[(id_high + 2) % armor_num_];

  // 物理 H→M→L 沿 id 递减方向时，模型角度公式不自洽，本帧跳过等下次更新
  if (z_next <= z_prev) return;

  ekf_.x[4]  = z_at_id_[id_high];
  ekf_.x[6]  = tools::limit_rad(ekf_.x[6] + id_high * 2 * CV_PI / armor_num_);
  ekf_.x[10] = -0.1;
  id = (id - id_high + armor_num_) % armor_num_;

  // 把被 rebuild 改过/影响的状态对应的 P 放回较大值，
  // 让 EKF 相信新观测、快速把 x[4] 等均到真值，避免 pitch 多秒慢收敛
  ekf_.P(4, 4) = 1.0;     // 高板 z
  ekf_.P(6, 6) = 0.4;     // 角度（手工平移过 id_high*2π/N）
  ekf_.P(7, 7) = 100.0;   // vyaw

  rebuilt_ = true;
}
void Target::update_ypda(const Armor_point & armor, int id)
{
  //观测jacobi
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);
  // Eigen::VectorXd R_dig{{4e-3, 4e-3, 1, 9e-2}};
  auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
  Eigen::VectorXd R_dig{
    {7e-3, 7e-3,
      log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 9e-1,
      log(std::abs(delta_angle) + 1) + 9e-3}};//log(std::abs(delta_angle) + 1) +
  // Eigen::VectorXd R_dig{{6e-2, 6e-2, 3e-1, 8e-2}};

  //测量过程噪声偏差的方差
  Eigen::MatrixXd R = R_dig.asDiagonal();

  // 定义非线性转换函数h: x -> z
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = h_armor_xyz(x, id);
    Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
    auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
    return {ypd[0], ypd[1], ypd[2], angle};
  };

  // 防止夹角求差出现异常值
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  const Eigen::VectorXd & ypd = armor.ypd_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;
  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0]}};  //获得观测量

  ekf_.update(z, H, R, h, z_subtract);
}

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_; }

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> _armor_xyza_list;

  for (int i = 0; i < armor_num_; i++) {
    auto angle = tools::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
    _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return _armor_xyza_list;
}

//根据安装规范判断半径有无分歧
bool Target::diverged() const
{
  auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
  auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;

  if (r_ok && l_ok) return false;

  //tools::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_.x[8], ekf_.x[9]);
  return true;
}

//根据update_count和diverged判断是否能用
bool Target::convergened()
{
  if (this->name != ArmorName::outpost && update_count_ > 3 && !this->diverged()) {
    is_converged_ = true;
  }

  //前哨站特殊判断
  if (this->name == ArmorName::outpost && update_count_ > 10 && !this->diverged()) {
    is_converged_ = true;
  }

  return is_converged_;
}

// 计算出装甲板中心的坐标（考虑长短轴）
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);
  auto use_h = (armor_num_ == 3) && (id == 1 || id == 2);
  // use_l_h = 0;

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];
  armor_z = (armor_num_ == 3) ? (use_h) ? rebuilt_ ? x[4] + x[10]*id : x[4] + x[10] : x[4] : armor_z;
  return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);
  auto use_h = (armor_num_ == 3) && (id == 1 || id == 2);

  // use_l_h = 0;

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

  auto dz_dh = (use_l_h || use_h) ? 1.0 : 0.0;
  dz_dh = dz_dh*((use_h && rebuilt_) ? id:1);
  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
  // clang-format on

  Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
    {                0,                 0,                 0, 1}
  };
  // clang-format on

  return H_armor_ypda * H_armor_xyza;
}

bool Target::checkinit() { return isinit; }

}  // namespace auto_aim

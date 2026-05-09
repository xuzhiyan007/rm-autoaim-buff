#include <rclcpp/rclcpp.hpp>
#include <fitting/srv/predict_energy.hpp>
#include <pose_transform/srv/compute_target_location.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <atomic>

using namespace std::chrono_literals;

class LittleEnergyClient : public rclcpp::Node
{
public:
  LittleEnergyClient() : Node("little_energy_client")
  {
    // 声明参数
    this->declare_parameter("call_rate", 20.0);      // Hz
    this->declare_parameter("dt", 1.0);              // 预测时间 (s)
    this->declare_parameter("bullet_speed", 22.0);  // 弹速 (m/s)

    dt_ = this->get_parameter("dt").as_double();
    call_rate_ = this->get_parameter("call_rate").as_double();
    bullet_speed_ = this->get_parameter("bullet_speed").as_double();

    // 创建服务客户端
    predict_client_ = this->create_client<fitting::srv::PredictEnergy>("/predict_energy");
    target_client_ = this->create_client<pose_transform::srv::ComputeTargetLocation>(
      "/compute_target_location");

    // 创建 /ene_energy_processor/energy_target 发布者 (x=pitch, y=yaw, z=shoot_flag)
    gimbal_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
      "/ene_energy_processor/energy_target", 10);

    // 创建 /debug_result 发布者 (x=result[0], y=result[1], z=result[2])
    debug_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
      "/ene_debug_result", 10);

    // 订阅 /joint_states，获取串口驱动发布的当前 yaw 和 pitch 值
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(rclcpp::KeepLast(1)),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
          if (msg->name[i] == "yaw_joint" && i < msg->position.size()) {
            current_yaw_.store(msg->position[i]);
          } else if (msg->name[i] == "pitch_joint" && i < msg->position.size()) {
            current_pitch_.store(msg->position[i]);
          }
        }
      });

    // 订阅 /detection_keypoints，更新检测有效性标志
    auto qos_detection = rclcpp::SensorDataQoS();
    qos_detection.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    detection_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/ene_detection_keypoints", qos_detection,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        const bool has = !msg->poses.empty();
        g_has_detection_.store(has);
        if (has) {
          g_last_detection_ns_.store(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now().time_since_epoch()).count());
        }
      });

    // 订阅 /robot_mode，仅当 mode==0(SMALL_ENERGY) 时激活本节点逻辑
    mode_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      "/robot_mode", rclcpp::QoS(rclcpp::KeepLast(1)),
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        robot_mode_.store(msg->data);
      });

    // 等待服务就绪
    RCLCPP_INFO(this->get_logger(), "Waiting for /predict_energy service...");
    while (!predict_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
        rclcpp::shutdown();
        throw std::runtime_error("Interrupted");
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
    }

    RCLCPP_INFO(this->get_logger(), "Waiting for /compute_target_location service...");
    while (!target_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
        rclcpp::shutdown();
        throw std::runtime_error("Interrupted");
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
    }

    // 创建 Timer 代替 while 循环
    auto period = std::chrono::duration<double>(1.0 / call_rate_);
    timer_ = this->create_wall_timer(period, std::bind(&LittleEnergyClient::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "Little Energy Client (async) started");
    RCLCPP_INFO(this->get_logger(), "dt = %.3f s, call_rate = %.1f Hz, bullet_speed = %.2f m/s",
                dt_, call_rate_, bullet_speed_);
  }

private:
  static constexpr double GRAVITY_LITTLE = 9.7915;
  static constexpr double FIRE_LATENCY = 0.075;
  static constexpr int64_t DETECTION_TIMEOUT_NS = 500'000'000LL;  // 500ms

  static constexpr int STATE_IDLE = 0;
  static constexpr int STATE_WAITING_PREDICT = 1;
  static constexpr int STATE_WAITING_COMPUTE = 2;

  // ========== Timer 回调：触发新一轮计算 ==========
  void onTimer()
  {
    if (robot_mode_.load() != 0) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "[little_energy_client] waiting for SMALL_ENERGY mode (current mode=%d)",
        static_cast<int>(robot_mode_.load()));
      return;
    }

    int expected = STATE_IDLE;
    if (!state_.compare_exchange_strong(expected, STATE_WAITING_PREDICT)) {
      // 上一轮还没算完，跳过本次触发
      return;
    }

    auto predict_req = std::make_shared<fitting::srv::PredictEnergy::Request>();
    predict_req->dt = dt_ + FIRE_LATENCY;

    RCLCPP_INFO(this->get_logger(), "Calling /predict_energy (async) with dt=%.3f", predict_req->dt);

    predict_client_->async_send_request(predict_req,
      [this](rclcpp::Client<fitting::srv::PredictEnergy>::SharedFuture future) {
        onPredictResponse(future);
      });
  }

  // ========== Predict 响应回调 ==========
  void onPredictResponse(rclcpp::Client<fitting::srv::PredictEnergy>::SharedFuture future)
  {
    auto predict_resp = future.get();
    if (!predict_resp->success) {
      RCLCPP_ERROR(this->get_logger(), "/predict_energy returned failure!");
      state_.store(STATE_IDLE);
      return;
    }

    RCLCPP_INFO(this->get_logger(),
               "/predict_energy success! delta_mu = %.6f rad", predict_resp->delta_mu);

    auto target_req = std::make_shared<pose_transform::srv::ComputeTargetLocation::Request>();
    // target_req->delta_mu = predict_resp->delta_mu;
    target_req->delta_mu = 0.517; //TODO

    RCLCPP_INFO(this->get_logger(),
               "Calling /compute_target_location (async) with delta_mu=%.6f",
               predict_resp->delta_mu);

    target_client_->async_send_request(target_req,
      [this](rclcpp::Client<pose_transform::srv::ComputeTargetLocation>::SharedFuture future) {
        onComputeResponse(future);
      });

    state_.store(STATE_WAITING_COMPUTE);
  }

  // ========== Compute 响应回调：弹道解算与发布 ==========
  void onComputeResponse(rclcpp::Client<pose_transform::srv::ComputeTargetLocation>::SharedFuture future)
  {
    // 无论成败，重置状态以便下一轮 Timer 可以触发
    state_.store(STATE_IDLE);

    auto target_resp = future.get();
    if (!target_resp->success) {
      RCLCPP_ERROR(this->get_logger(), "/compute_target_location returned failure!");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "/compute_target_location success!");
    RCLCPP_INFO(this->get_logger(),
               "Target position: [%.3f, %.3f, %.3f]",
               target_resp->p_final.x, target_resp->p_final.y, target_resp->p_final.z);

    // result: [x(左右), y(高度), z(前方)] - 枪管坐标系 A' 下的坐标（z轴平行枪管）
    double px_g = target_resp->p_final.x;
    double py_g = target_resp->p_final.y;
    double pz_g = target_resp->p_final.z;

    // 枪管坐标系 A' -> A系（z轴正方向为绝对前方）
    double motor_pitch = current_pitch_.load();
    double motor_yaw   = current_yaw_.load();

    // A' 系是 A 系先绕 y 轴旋转 yaw，再绕 x 轴旋转 pitch 得到
    // 使用 tf2 四元数：先构建从 A 到 A' 的旋转，然后求逆
    tf2::Quaternion q_pitch, q_yaw;
    q_pitch.setRPY(motor_pitch, 0.0, 0.0);  // 绕 x 轴旋转 pitch
    q_yaw.setRPY(0.0, 0.0, 0.0);      // 绕 y 轴旋转 yaw

    // q_pitch.setRPY(0.0, -motor_pitch, 0.0);  // 绕 x 轴旋转 pitch
    // q_yaw.setRPY(0.0, 0.0, -motor_yaw);      // 绕 y 轴旋转 yaw

    tf2::Quaternion q_A_to_Ap = q_yaw * q_pitch;
    tf2::Quaternion q_Ap_to_A = q_A_to_Ap.inverse();

    // 将枪管坐标系 A' 下的目标点变换到 A 系
    tf2::Vector3 p_Ap(px_g, py_g, pz_g);
    tf2::Vector3 p_A = tf2::quatRotate(q_Ap_to_A, p_Ap);

    double px = p_A.x();
    double py = p_A.y();
    double pz = p_A.z();

    // 发布调试结果到 /debug_result
    geometry_msgs::msg::Vector3 debug_msg;
    debug_msg.x = px;
    debug_msg.y = py;
    debug_msg.z = pz;
    debug_pub_->publish(debug_msg);
    //END
    double dist = std::sqrt(px * px + pz * pz);  // 水平距离

    double yaw = 0.0;
    bool yaw_ok = SolveYaw(pz_g, px_g, yaw);

    double pitch = 0.0;
    int shoot_flag = 0;
    bool pitch_ok = Solve_pitch_local(bullet_speed_, dist, py, pitch);

    double dt_new = dt_;  // 默认保持原 dt
    if (pitch_ok) {
      // dt' = 飞行时间，作为下次预测的超前量
      dt_new = dist / (bullet_speed_ * std::cos(pitch));
      shoot_flag = 1;
      RCLCPP_INFO(this->get_logger(),
                  "Solve_pitch OK: pitch=%.4f rad, yaw=%.4f rad, dt'=%.4f s",
                  pitch, yaw, dt_new);

      // 发布 /gimbal_command (x=pitch, y=yaw, z=shoot_flag)
      geometry_msgs::msg::Vector3 cmd;
      cmd.x = pitch;
      cmd.y = motor_yaw + yaw;
      cmd.z = 2.0;
      gimbal_pub_->publish(cmd);

    } else {
      RCLCPP_WARN(this->get_logger(), "Solve_pitch failed, dt not updated");
    }

    // 将解算到的飞行时间作为下一次 onTimer 的 dt
    dt_ = dt_new;
  }

  // ======================== 弹道解算 (复用自 energy_fire_ctrl.cpp) ========================
  bool Solve_pitch_local(double bullet_speed, double target_distance, double target_h, double& pitch)
  {
    target_h = -target_h;
    double aimed_h = target_h, h = 0.0, dh = 0.0, tmp_pitch = 0.0, t = 0.0;
    for (int i = 0; i < 20; i++) {
      tmp_pitch = atan2(aimed_h, target_distance);
      if (tmp_pitch > 80.0 * M_PI / 180.0 || tmp_pitch < -80.0 * M_PI / 180.0) return false;
      t = target_distance / (bullet_speed * cos(tmp_pitch));
      h = bullet_speed * sin(tmp_pitch) * t - GRAVITY_LITTLE * t * t / 2.0;
      if (t > 10.0) return false;
      dh = target_h - h;
      aimed_h += dh;
      if (fabs(dh) < 0.001) break;
    }
    if (fabs(dh) > 0.01) return false;

    // 若 /detection_keypoints 无有效数据，或超过 500ms 未收到有效检测，禁止解算
    if (!g_has_detection_.load()) return false;
    {
      auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
      if (now_ns - g_last_detection_ns_.load() > DETECTION_TIMEOUT_NS) return false;
    }

    pitch = -tmp_pitch;
    return true;
  }

  bool SolveYaw(double pz_g, double px_g, double& yaw)
  {
    yaw = -atan2(px_g, pz_g);
    return true;
  }

  // ROS 接口
  rclcpp::Client<fitting::srv::PredictEnergy>::SharedPtr predict_client_;
  rclcpp::Client<pose_transform::srv::ComputeTargetLocation>::SharedPtr target_client_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gimbal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 参数
  double dt_;
  double call_rate_;
  double bullet_speed_;

  // 外部状态（由订阅回调更新）
  std::atomic<uint8_t> robot_mode_{2};
  std::atomic<double> current_yaw_{0.0};
  std::atomic<double> current_pitch_{0.0};
  std::atomic<bool> g_has_detection_{false};
  std::atomic<int64_t> g_last_detection_ns_{0};

  // 状态机
  std::atomic<int> state_{STATE_IDLE};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<LittleEnergyClient>();
    rclcpp::spin(node);   // ← 唯一的一处 spin
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("little_energy_client"), "Exception: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}

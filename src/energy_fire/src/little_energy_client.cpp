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
double fire_latency = 0.075;

// 全局标志：/detection_keypoints 是否有有效数据
std::atomic<bool> g_has_detection{false};
// 最后一次收到有效检测的时间点（steady_clock nanoseconds）
std::atomic<int64_t> g_last_detection_ns{0};
constexpr int64_t DETECTION_TIMEOUT_NS = 500'000'000LL;  // 500ms

// 当前机器人模式：0=SMALL_ENERGY, 1=BIG_ENERGY, 2=ARMOR（默认2，即不激活）
std::atomic<uint8_t> g_robot_mode{2};

/**
 * @brief 同步执行一次完整的服务调用链：
 *        predict_energy -> compute_target_location
 * @param node             ROS2 节点指针
 * @param predict_client   /predict_energy 服务客户端
 * @param target_client    /compute_target_location 服务客户端
 * @param dt               预测时间 (s)
 * @return 包含 p_final.x, p_final.y, p_final.z 的向量；失败时返回空向量
 */
std::vector<float> runOnce(
    rclcpp::Node::SharedPtr node,
    rclcpp::Client<fitting::srv::PredictEnergy>::SharedPtr predict_client,
    rclcpp::Client<pose_transform::srv::ComputeTargetLocation>::SharedPtr target_client,
    double dt)
{
    // ---- 第一步：同步调用 /predict_energy ----
    auto predict_req = std::make_shared<fitting::srv::PredictEnergy::Request>();
    dt = dt + fire_latency; 

    predict_req->dt = dt;

    RCLCPP_INFO(node->get_logger(), "Calling /predict_energy (sync) with dt=%.3f", dt);

    auto predict_future = predict_client->async_send_request(predict_req);

    if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), predict_future)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "/predict_energy call failed or timed out!");
        return {};
    }

    auto predict_resp = predict_future.get();
    if (!predict_resp->success) {
        RCLCPP_ERROR(node->get_logger(), "/predict_energy returned failure!");
        return {};
    }

    RCLCPP_INFO(node->get_logger(),
               "/predict_energy success! delta_mu = %.6f rad", predict_resp->delta_mu);

    // ---- 第二步：同步调用 /compute_target_location ----
    auto target_req = std::make_shared<pose_transform::srv::ComputeTargetLocation::Request>();
    //target_req->delta_mu = predict_resp->delta_mu;
    target_req->delta_mu = 0.517; //TODO
    RCLCPP_INFO(node->get_logger(),
               "Calling /compute_target_location (sync) with delta_mu=%.6f",
               predict_resp->delta_mu);

    auto target_future = target_client->async_send_request(target_req);

    if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), target_future)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "/compute_target_location call failed or timed out!");
        return {};
    }

    auto target_resp = target_future.get();
    if (!target_resp->success) {
        RCLCPP_ERROR(node->get_logger(), "/compute_target_location returned failure!");
        return {};
    }

    RCLCPP_INFO(node->get_logger(), "/compute_target_location success!");
    RCLCPP_INFO(node->get_logger(),
               "Target position: [%.3f, %.3f, %.3f]",
               target_resp->p_final.x, target_resp->p_final.y, target_resp->p_final.z);
    return {static_cast<float>(target_resp->p_final.x),
            static_cast<float>(target_resp->p_final.y),
            static_cast<float>(target_resp->p_final.z)};
}

// ======================== 弹道解算 (复用自 energy_fire_ctrl.cpp) ========================
constexpr double GRAVITY_LITTLE = 9.7915;

bool Solve_pitch_local(double bullet_speed, double target_distance, double target_h, double& pitch) {
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
    if (!g_has_detection.load()) return false;
    {
        auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if (now_ns - g_last_detection_ns.load() > DETECTION_TIMEOUT_NS) return false;
    }
    pitch = -tmp_pitch;
    return true;
}

bool SolveYaw(double pz_g, double px_g, double& yaw) {
    yaw = -atan2(px_g, pz_g);
    return true;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("little_energy_client");

    // 声明参数
    node->declare_parameter("call_rate", 20.0);      // Hz
    node->declare_parameter("dt", 1.0);              // 预测时间 (s)
    node->declare_parameter("bullet_speed", 22.0);  // 弹速 (m/s)

    double dt           = node->get_parameter("dt").as_double();
    double call_rate    = node->get_parameter("call_rate").as_double();
    double bullet_speed = node->get_parameter("bullet_speed").as_double();

    // 创建服务客户端
    auto predict_client = node->create_client<fitting::srv::PredictEnergy>("/predict_energy");
    auto target_client  = node->create_client<pose_transform::srv::ComputeTargetLocation>(
        "/compute_target_location");

    // 创建 /gimbal_command 发布者 (x=pitch, y=yaw, z=shoot_flag)
    auto gimbal_pub = node->create_publisher<geometry_msgs::msg::Vector3>(
        "/ene_energy_processor/energy_target", 10);


    // 创建 /debug_result 发布者 (x=result[0], y=result[1], z=result[2])
    auto debug_pub = node->create_publisher<geometry_msgs::msg::Vector3>(
        "/ene_debug_result", 10);

    // 订阅 /joint_states，获取串口驱动发布的当前 yaw 和 pitch 值
    std::atomic<double> current_yaw{0.0};
    std::atomic<double> current_pitch{0.0};
    auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::QoS(rclcpp::KeepLast(1)),
        [&current_yaw, &current_pitch](const sensor_msgs::msg::JointState::SharedPtr msg) {
            for (size_t i = 0; i < msg->name.size(); ++i) {
                if (msg->name[i] == "yaw_joint" && i < msg->position.size()) {
                    current_yaw.store(msg->position[i]);
                } else if (msg->name[i] == "pitch_joint" && i < msg->position.size()) {
                    current_pitch.store(msg->position[i]);
                }
            }
        });

    // 订阅 /detection_keypoints，更新检测有效性标志
    auto qos_detection = rclcpp::SensorDataQoS();
    qos_detection.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    auto detection_sub = node->create_subscription<geometry_msgs::msg::PoseArray>(
        "/ene_detection_keypoints", qos_detection,
        [](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
            const bool has = !msg->poses.empty();
            g_has_detection.store(has);
            if (has) {
                g_last_detection_ns.store(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count());
            }
        });

    // 订阅 /robot_mode，仅当 mode==0(SMALL_ENERGY) 时激活本节点逻辑
    auto mode_sub = node->create_subscription<std_msgs::msg::UInt8>(
        "/robot_mode", rclcpp::QoS(rclcpp::KeepLast(1)),
        [](const std_msgs::msg::UInt8::SharedPtr msg) {
            g_robot_mode.store(msg->data);
        });

    // 等待服务就绪
    RCLCPP_INFO(node->get_logger(), "Waiting for /predict_energy service...");
    while (!predict_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service.");
            rclcpp::shutdown();
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Service not available, waiting...");
    }

    RCLCPP_INFO(node->get_logger(), "Waiting for /compute_target_location service...");
    while (!target_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service.");
            rclcpp::shutdown();
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Service not available, waiting...");
    }

    RCLCPP_INFO(node->get_logger(), "Little Energy Client (sync) started");
    RCLCPP_INFO(node->get_logger(), "dt = %.3f s, call_rate = %.1f Hz, bullet_speed = %.2f m/s",
                dt, call_rate, bullet_speed);

    rclcpp::Rate rate(call_rate);

    while (rclcpp::ok()) {
        // 处理订阅回调（含 /robot_mode、/joint_states、/ene_detection_keypoints）
        rclcpp::spin_some(node);

        // 非小能量机关模式（mode != 0）时跳过，保持待机
        if (g_robot_mode.load() != 0) {
            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 3000,
                "[little_energy_client] waiting for SMALL_ENERGY mode (current mode=%d)",
                static_cast<int>(g_robot_mode.load()));
            rate.sleep();
            continue;
        }

        auto result = runOnce(node, predict_client, target_client, dt);

        if (!result.empty()) {
            // result: [x(左右), y(高度), z(前方)] - 枪管坐标系 A' 下的坐标（z轴平行枪管）
            double px_g = result[0], py_g = result[1], pz_g = result[2];

            // 枪管坐标系 A' -> A系（z轴正方向为绝对前方）
            // 从 /joint_states 获取当前电机的 yaw 和 pitch 值
            double motor_pitch = current_pitch.load();
            double motor_yaw   = current_yaw.load();

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
            debug_pub->publish(debug_msg);
            //END
            double dist = std::sqrt(px * px + pz * pz);  // 水平距离

            double yaw = 0.0;
            bool yaw_ok = SolveYaw(pz_g, px_g, yaw);

            double pitch = 0.0;
            int shoot_flag = 0;
            bool pitch_ok = Solve_pitch_local(bullet_speed, dist, py, pitch);

            double dt_new = dt;  // 默认保持原 dt
            if (pitch_ok) {
                // dt' = 飞行时间，作为下次预测的超前量
                dt_new = dist / (bullet_speed * std::cos(pitch));
                shoot_flag = 1;
                RCLCPP_INFO(node->get_logger(),
                            "Solve_pitch OK: pitch=%.4f rad, yaw=%.4f rad, dt'=%.4f s",
                            pitch, yaw, dt_new);

            // 发布 /gimbal_command (x=pitch, y=yaw, z=shoot_flag)
            geometry_msgs::msg::Vector3 cmd;
            cmd.x = pitch;
            cmd.y = motor_yaw + yaw;
            cmd.z = 2.0;
            gimbal_pub->publish(cmd);


            } else {
                RCLCPP_WARN(node->get_logger(), "Solve_pitch failed, dt not updated");
            }

            // 将解算到的飞行时间作为下一次 runOnce 的 dt
            dt = dt_new;
        }

        rate.sleep();
    } 

    rclcpp::shutdown();
    return 0;
}

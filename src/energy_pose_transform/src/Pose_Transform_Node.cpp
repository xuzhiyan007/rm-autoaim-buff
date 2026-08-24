#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <mutex>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>

// 全局变量（按照现有代码风格）
rclcpp::Node::SharedPtr g_node = nullptr;
rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr g_transformed_poses_pub = nullptr;
rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr g_selected_poses_pub = nullptr;

// message_filters 订阅者（保留原有变量名风格）
std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseArray>> g_poses_sub = nullptr;
std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::JointState>> g_joint_state_sub = nullptr;

// /pnp_poses_selected_a 的独立同步器
std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseArray>> g_selected_poses_sub = nullptr;
std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::JointState>> g_joint_state_sub2 = nullptr;

// 同步器
using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    geometry_msgs::msg::PoseArray,
    sensor_msgs::msg::JointState>;
std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> g_sync = nullptr;
std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> g_sync_selected = nullptr;

// 数据缓存
geometry_msgs::msg::PoseArray::SharedPtr g_latest_poses = nullptr;
std::mutex g_data_mutex;

// 变换参数
double g_translation_x1 = 0.0, g_translation_y1 = 0.0, g_translation_z1 = 0.0;
double g_theta = 0.0;  // pitch angle (俯仰角)
double g_gamma = 0.0;  // yaw angle (偏航角)
int g_time_tolerance_ms = 100;

// 简单的YAML解析函数 - 读取 translation.x1, y1, z1
bool loadTranslationFromYAML(const std::string& yaml_file, double& x1, double& y1, double& z1)
{
    std::ifstream file(yaml_file);
    if (!file.is_open()) {
        return false;
    }

    std::string line;
    bool in_translation_section = false;
    
    while (std::getline(file, line)) {
        // 移除前导空格，但先记录是否有缩进
        size_t start = line.find_first_not_of(" \t");
        if (start == std::string::npos) continue;
        bool has_indent = (start > 0);
        line = line.substr(start);
        
        // 跳过注释行
        if (line.empty() || line[0] == '#') continue;
        
        // 检测 translation: 节
        if (line.find("translation:") == 0) {
            in_translation_section = true;
            continue;
        }
        
        // 如果在 translation 节中，解析参数
        if (in_translation_section) {
            if (line.find(':') != std::string::npos && has_indent) {
                // 有缩进，是子项，解析键值（格式为 "key: value ..."）
                size_t colon_pos = line.find(':');
                std::string key = line.substr(0, colon_pos);
                std::istringstream val_ss(line.substr(colon_pos + 1));
                double value;
                if (val_ss >> value) {
                    if (key == "x1") x1 = value;
                    else if (key == "y1") y1 = value;
                    else if (key == "z1") z1 = value;
                }
            } else if (!has_indent) {
                // 无缩进，说明新节开始
                in_translation_section = false;
            }
        }
    }
    
    file.close();
    return true;
}

// 四元数乘法函数
void quaternionMultiply(const double q1[4], const double q2[4], double result[4]) {
    result[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    result[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    result[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    result[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

// 从A系到C系的坐标变换函数（theta 由调用方传入，避免使用过期的全局值）
geometry_msgs::msg::Pose transformPoseFromAToC(const geometry_msgs::msg::Pose& pose_a, double theta, double gamma)
{
    geometry_msgs::msg::Pose pose_c;

    // 从A系到C系的位置变换（严格按照Axes_Transform.md公式）
    double x_a = pose_a.position.x;
    double y_a = pose_a.position.y;
    double z_a = pose_a.position.z;

    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);
    double cos_gamma = std::cos(gamma);
    double sin_gamma = std::sin(gamma);

    // 完整的变换公式：
    // x_C = cos(γ)*x_A + sin(γ)*sin(θ)*y_A + sin(γ)*cos(θ)*z_A + (x₁*cos(γ) + z₁*sin(γ))
    pose_c.position.x = cos_gamma * x_a +
                       sin_gamma * sin_theta * y_a +
                       sin_gamma * cos_theta * z_a +
                       (g_translation_x1 * cos_gamma + g_translation_z1 * sin_gamma);

    // y_C = cos(θ)*y_A - sin(θ)*z_A + y₁
    pose_c.position.y = cos_theta * y_a -
                       sin_theta * z_a +
                       g_translation_y1;

    // z_C = -sin(γ)*x_A + cos(γ)*sin(θ)*y_A + cos(γ)*cos(θ)*z_A + (-x₁*sin(γ) + z₁*cos(γ))
    pose_c.position.z = -sin_gamma * x_a +
                       cos_gamma * sin_theta * y_a +
                       cos_gamma * cos_theta * z_a +
                       (-g_translation_x1 * sin_gamma + g_translation_z1 * cos_gamma);

    // 姿态变换：q_C = q_CB ⊗ q_BA ⊗ q_A
    // q_BA = [cos(θ/2), sin(θ/2), 0, 0]  (绕x轴旋转θ)
    // q_CB = [cos(γ/2), 0, sin(γ/2), 0]  (绕y轴旋转γ)

    // 输入四元数 (从A系姿态)
    double q_a[4] = {pose_a.orientation.w, pose_a.orientation.x, pose_a.orientation.y, pose_a.orientation.z};

    // q_BA: 绕x轴旋转θ
    double q_ba[4] = {std::cos(theta/2.0), std::sin(theta/2.0), 0.0, 0.0};

    // q_CB: 绕y轴旋转γ
    double q_cb[4] = {std::cos(gamma/2.0), 0.0, std::sin(gamma/2.0), 0.0};

    // 四元数乘法：q_temp = q_BA ⊗ q_A
    double q_temp[4];
    quaternionMultiply(q_ba, q_a, q_temp);

    // 四元数乘法：q_c = q_CB ⊗ q_temp
    double q_c[4];
    quaternionMultiply(q_cb, q_temp, q_c);

    // 归一化四元数
    double norm = std::sqrt(q_c[0]*q_c[0] + q_c[1]*q_c[1] + q_c[2]*q_c[2] + q_c[3]*q_c[3]);
    if (norm > 1e-9) {
        q_c[0] /= norm;
        q_c[1] /= norm;
        q_c[2] /= norm;
        q_c[3] /= norm;
    }

    pose_c.orientation.w = q_c[0];
    pose_c.orientation.x = q_c[1];
    pose_c.orientation.y = q_c[2];
    pose_c.orientation.z = q_c[3];

    return pose_c;
}

// 从 JointState 消息中提取角度（辅助函数，保留原有函数名）
void jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg,
                        double& out_theta, double& out_gamma)
{
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        if (msg->name[i] == "pitch_joint") {
            out_theta = -msg->position[i];
        } else if (msg->name[i] == "yaw_joint") {
            out_gamma = -msg->position[i];
        }
    }
}

// 位姿数据回调函数 - 时间同步版本，使用相机发布时刻对应的 JointState
void posesCallback(const geometry_msgs::msg::PoseArray::ConstSharedPtr& poses_msg,
                   const sensor_msgs::msg::JointState::ConstSharedPtr& joint_msg)
{
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_latest_poses = std::make_shared<geometry_msgs::msg::PoseArray>(*poses_msg);

    // 从同步的 JointState 中提取角度（与 poses_msg 时间戳对齐）
    double sync_theta = g_theta;
    double sync_gamma = g_gamma;
    jointStateCallback(joint_msg, sync_theta, sync_gamma);

    // 更新全局角度 TODO
    g_theta = sync_theta;  //pitch
    g_gamma = sync_gamma;  //yaw

    // 对接收到的 /pnp_poses 进行变换，使用同步时刻的角度
    geometry_msgs::msg::PoseArray transformed_poses;
    transformed_poses.header = g_latest_poses->header;  // 保留原始时间戳
    transformed_poses.header.frame_id = "c_frame";  // 标记为C坐标系

    for (size_t i = 0; i < g_latest_poses->poses.size(); ++i) {
        const auto& pose_a = g_latest_poses->poses[i];
        geometry_msgs::msg::Pose pose_c = transformPoseFromAToC(pose_a, sync_theta, sync_gamma);
        transformed_poses.poses.push_back(pose_c);
    }

    g_transformed_poses_pub->publish(transformed_poses);

    RCLCPP_INFO(g_node->get_logger(),
                "A2C solving angles: theta=%.4f rad, gamma=%.4f rad, transformed %zu poses",
                sync_theta, sync_gamma, g_latest_poses->poses.size());
}

// 选中位姿回调函数 - 将 /pnp_poses_selected_a 变换到C系，发布为 /pnp_poses_select
void selectedPosesCallback(const geometry_msgs::msg::PoseArray::ConstSharedPtr& poses_msg,
                            const sensor_msgs::msg::JointState::ConstSharedPtr& joint_msg)
{
    std::lock_guard<std::mutex> lock(g_data_mutex);

    double sync_theta = g_theta;
    double sync_gamma = g_gamma;
    jointStateCallback(joint_msg, sync_theta, sync_gamma);

    // 同步更新全局角度，保持 g_theta/g_gamma 为最新值（避免主链路停发时使用过期pitch）
    g_theta = sync_theta;
    g_gamma = sync_gamma;

    geometry_msgs::msg::PoseArray transformed_poses;
    transformed_poses.header = poses_msg->header;
    transformed_poses.header.frame_id = "c_frame";

    for (size_t i = 0; i < poses_msg->poses.size(); ++i) {
        const auto& pose_a = poses_msg->poses[i];
        geometry_msgs::msg::Pose pose_c = transformPoseFromAToC(pose_a, sync_theta, sync_gamma);
        transformed_poses.poses.push_back(pose_c);
    }

    g_selected_poses_pub->publish(transformed_poses);

    static bool logged_first_selected_a2c_success = false;
    if (!logged_first_selected_a2c_success) {
        RCLCPP_INFO(g_node->get_logger(),
                    "Selected A2C first success: theta=%.4f rad, gamma=%.4f rad",
                    sync_theta, sync_gamma);
        logged_first_selected_a2c_success = true;
    } else {
        RCLCPP_DEBUG(g_node->get_logger(),
                    "Selected A2C synced: theta=%.4f rad, gamma=%.4f rad", sync_theta, sync_gamma);
    }
}

int main(int argc, char** argv)
{
    // 1. 初始化 ROS2
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("coordinate_a2c_node");

    // 2. 从配置文件读取平移参数
    try {
        std::string package_share_dir = ament_index_cpp::get_package_share_directory("energy_pose_transform");
        std::string config_file = package_share_dir + "/config/transform_params.yaml";
        
        RCLCPP_INFO(g_node->get_logger(), "Loading config from: %s", config_file.c_str());
        
        if (loadTranslationFromYAML(config_file, g_translation_x1, g_translation_y1, g_translation_z1)) {
            RCLCPP_INFO(g_node->get_logger(), "Successfully loaded translation parameters from config file");
        } else {
            RCLCPP_WARN(g_node->get_logger(), "Failed to load config file, using default values (0.0, 0.0, 0.0)");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(g_node->get_logger(), "Error loading config: %s. Using default values.", e.what());
    }

    // 3. 获取其他参数
    g_theta = 0.0;
    g_gamma = 0.0;
    g_time_tolerance_ms = g_node->declare_parameter("time_tolerance_ms", 100);

    RCLCPP_INFO(g_node->get_logger(), "Coordinate A2C Transform Node started");
    RCLCPP_INFO(g_node->get_logger(), "Translation: x1=%.3f, y1=%.3f, z1=%.3f",
                g_translation_x1, g_translation_y1, g_translation_z1);
    RCLCPP_INFO(g_node->get_logger(), "Angles: theta=%.3f rad, gamma=%.3f rad", g_theta, g_gamma);

    // 4. 创建 message_filters 订阅者 - 时间同步订阅
    g_poses_sub = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseArray>>(
        g_node, "/ene_pnp_poses", rmw_qos_profile_default);

    g_joint_state_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::JointState>>(
        g_node, "/joint_states", rmw_qos_profile_default);

    // 创建 ApproximateTime 同步器 - 使用相机发布时刻对应的 JointState
    g_sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(30), *g_poses_sub, *g_joint_state_sub);
    g_sync->registerCallback(posesCallback);

    // 第二同步器：订阅 /pnp_poses_selected_a + /joint_states，发布 /pnp_poses_select
    g_selected_poses_sub = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseArray>>(
        g_node, "/ene_pnp_poses_selected_a", rmw_qos_profile_default);
    g_joint_state_sub2 = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::JointState>>(
        g_node, "/joint_states", rmw_qos_profile_default);
    g_sync_selected = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(30), *g_selected_poses_sub, *g_joint_state_sub2);
    g_sync_selected->registerCallback(selectedPosesCallback);

    RCLCPP_INFO(g_node->get_logger(), "Using ApproximateTime synchronizer for pose-joint alignment");

    // 5. 创建发布者（改为发布到 /pnp_poses_ground，使用默认QoS）
    g_transformed_poses_pub = g_node->create_publisher<geometry_msgs::msg::PoseArray>(
        "/ene_pnp_poses_ground", 10);

    // 发布选中 pose 的C系位姿
    g_selected_poses_pub = g_node->create_publisher<geometry_msgs::msg::PoseArray>(
        "/ene_pnp_poses_select", 10);

    RCLCPP_INFO(g_node->get_logger(), "Subscribed to: /ene_pnp_poses + /joint_states");
    RCLCPP_INFO(g_node->get_logger(), "Subscribed to: /ene_pnp_poses_selected_a + /joint_states");
    RCLCPP_INFO(g_node->get_logger(), "Publishing to: /ene_pnp_poses_ground");
    RCLCPP_INFO(g_node->get_logger(), "Publishing to: /ene_pnp_poses_select");

    // 6. 运行节点
    rclcpp::spin(g_node);

    // 7. 清理资源
    rclcpp::shutdown();
    return 0;
}
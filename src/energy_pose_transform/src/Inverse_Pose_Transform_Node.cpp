#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <mutex>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <energy_pose_transform/srv/compute_target_location.hpp>

// 全局变量（按照现有代码风格）
rclcpp::Node::SharedPtr g_node = nullptr;
rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr g_poses_sub = nullptr;
rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr g_selected_poses_sub = nullptr;
rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr g_transformed_poses_pub = nullptr;
rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr g_joint_state_sub = nullptr;
rclcpp::Service<energy_pose_transform::srv::ComputeTargetLocation>::SharedPtr g_compute_service = nullptr;
rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr g_target_location_pub = nullptr;

// 数据缓存
geometry_msgs::msg::PoseArray::SharedPtr g_latest_poses = nullptr;
geometry_msgs::msg::PoseArray::SharedPtr g_latest_selected_poses = nullptr;  // /pnp_poses_select 缓存
std::mutex g_data_mutex;

// 变换参数（逆变换用的是枪管相对pitch坐标系的偏移）
double g_translation_x1_prime = 0.0, g_translation_y1_prime = 0.0, g_translation_z1_prime = 0.0;
double g_theta = 0.0;  // pitch angle (俯仰角)
double g_gamma = 0.0;  // yaw angle (偏航角)
int g_time_tolerance_ms = 100;

// 简单的YAML解析函数 - 读取 inverse_translation.x1_prime, y1_prime, z1_prime
bool loadInverseTranslationFromYAML(const std::string& yaml_file, double& x1_prime, double& y1_prime, double& z1_prime)
{
    std::ifstream file(yaml_file);
    if (!file.is_open()) {
        return false;
    }

    std::string line;
    bool in_inverse_translation_section = false;
    
    while (std::getline(file, line)) {
        // 移除前导空格
        size_t start = line.find_first_not_of(" \t");
        if (start == std::string::npos) continue;
        line = line.substr(start);
        
        // 跳过注释行
        if (line.empty() || line[0] == '#') continue;
        
        // 检测 inverse_translation: 节
        if (line.find("inverse_translation:") == 0) {
            in_inverse_translation_section = true;
            continue;
        }
        
        // 如果在 inverse_translation 节中，解析参数
        if (in_inverse_translation_section) {
            // 遇到新节则退出
            if (line.find(':') != std::string::npos && line.find_first_of(" \t") == 0) {
                // 检查是否是子项（有缩进）
                std::istringstream iss(line);
                std::string key;
                double value;
                char colon;
                
                if (iss >> key >> colon >> value && colon == ':') {
                    if (key == "x1_prime") x1_prime = value;
                    else if (key == "y1_prime") y1_prime = value;
                    else if (key == "z1_prime") z1_prime = value;
                }
            } else if (line[0] != ' ' && line[0] != '\t') {
                // 无缩进，说明新节开始
                in_inverse_translation_section = false;
            }
        }
    }
    
    file.close();
    return true;
}

// 从C系到A系的坐标逆变换函数（基于Axes_Transform.md的逆变换公式）
geometry_msgs::msg::Pose transformPoseFromCToA(const geometry_msgs::msg::Pose& pose_c, double gamma)
{
    geometry_msgs::msg::Pose pose_a;

    // 从C系到A系的位置逆变换（严格按照Axes_Transform.md的逆变换公式）
    double x_c = pose_c.position.x;
    double y_c = pose_c.position.y;
    double z_c = pose_c.position.z;

    double cos_theta = std::cos(g_theta);
    double sin_theta = std::sin(g_theta);
    double cos_gamma = std::cos(gamma);
    double sin_gamma = std::sin(gamma);

    // 计算中间变量
    // Δx = x_C - (x₁'*cos(γ) + z₁'*sin(γ))
    double delta_x = x_c - (g_translation_x1_prime * cos_gamma + g_translation_z1_prime * sin_gamma);
    
    // Δy = y_C - y₁'
    double delta_y = y_c - g_translation_y1_prime;
    
    // Δz = z_C - (-x₁'*sin(γ) + z₁'*cos(γ))
    double delta_z = z_c - (-g_translation_x1_prime * sin_gamma + g_translation_z1_prime * cos_gamma);

    // 逆变换公式（应用R_CA的转置）：
    // x_A = cos(γ)*Δx - sin(γ)*Δz
    pose_a.position.x = cos_gamma * delta_x - sin_gamma * delta_z;

    // y_A = sin(γ)*sin(θ)*Δx + cos(θ)*Δy + cos(γ)*sin(θ)*Δz
    pose_a.position.y = sin_gamma * sin_theta * delta_x + 
                       cos_theta * delta_y + 
                       cos_gamma * sin_theta * delta_z;

    // z_A = sin(γ)*cos(θ)*Δx - sin(θ)*Δy + cos(γ)*cos(θ)*Δz
    pose_a.position.z = sin_gamma * cos_theta * delta_x - 
                       sin_theta * delta_y + 
                       cos_gamma * cos_theta * delta_z;

    // 姿态逆变换：q_AC = conjugate(q_CA)
    // q_CA = [cos(γ/2)*cos(θ/2), cos(γ/2)*sin(θ/2), sin(γ/2)*cos(θ/2), -sin(γ/2)*sin(θ/2)]
    // q_AC = [w, -x, -y, -z]
    
    // 输入四元数 (从C系姿态)
    double q_c[4] = {pose_c.orientation.w, pose_c.orientation.x, pose_c.orientation.y, pose_c.orientation.z};

    // 计算 q_CA (forward transform quaternion)
    double cos_theta_half = std::cos(g_theta / 2.0);
    double sin_theta_half = std::sin(g_theta / 2.0);
    double cos_gamma_half = std::cos(gamma / 2.0);
    double sin_gamma_half = std::sin(gamma / 2.0);

    double q_ca[4] = {
        cos_gamma_half * cos_theta_half,
        cos_gamma_half * sin_theta_half,
        sin_gamma_half * cos_theta_half,
        -sin_gamma_half * sin_theta_half
    };

    // q_AC = conjugate(q_CA) = [w, -x, -y, -z]
    double q_ac[4] = {
        q_ca[0],
        -q_ca[1],
        -q_ca[2],
        -q_ca[3]
    };

    // 应用逆旋转: q_A = q_AC ⊗ q_C
    // 四元数乘法
    double q_a[4];
    q_a[0] = q_ac[0]*q_c[0] - q_ac[1]*q_c[1] - q_ac[2]*q_c[2] - q_ac[3]*q_c[3];
    q_a[1] = q_ac[0]*q_c[1] + q_ac[1]*q_c[0] + q_ac[2]*q_c[3] - q_ac[3]*q_c[2];
    q_a[2] = q_ac[0]*q_c[2] - q_ac[1]*q_c[3] + q_ac[2]*q_c[0] + q_ac[3]*q_c[1];
    q_a[3] = q_ac[0]*q_c[3] + q_ac[1]*q_c[2] - q_ac[2]*q_c[1] + q_ac[3]*q_c[0];

    // 归一化四元数
    double norm = std::sqrt(q_a[0]*q_a[0] + q_a[1]*q_a[1] + q_a[2]*q_a[2] + q_a[3]*q_a[3]);
    if (norm > 1e-9) {
        q_a[0] /= norm;
        q_a[1] /= norm;
        q_a[2] /= norm;
        q_a[3] /= norm;
    }

    pose_a.orientation.w = q_a[0];
    pose_a.orientation.x = q_a[1];
    pose_a.orientation.y = q_a[2];
    pose_a.orientation.z = q_a[3];

    return pose_a;
}

void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(g_data_mutex);

    double new_theta = g_theta;
    double new_gamma = g_gamma;

    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        if (msg->name[i] == "pitch_joint") {

            //在这里改订阅的pitch 和yaw
            new_theta = -msg->position[i];
            //new_theta = 0.0;
        } else if (msg->name[i] == "yaw_joint") {
            new_gamma = -msg->position[i];
            //new_gamma = 0.0;
        }
    }

    g_theta = new_theta;
    g_gamma = new_gamma;
}

// 位姿数据回调函数
void posesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_latest_poses = msg;

    // 对接收到的 /pnp_poses_ground 进行逆变换，使用全局参数 g_gamma
    geometry_msgs::msg::PoseArray transformed_poses;
    transformed_poses.header = g_latest_poses->header;
    // transformed_poses.header.frame_id = "a_frame";  // 标记为A坐标系（枪管坐标系）
    transformed_poses.header.frame_id = "camera_frame";
    for (size_t i = 0; i < g_latest_poses->poses.size(); ++i) {
        const auto& pose_c = g_latest_poses->poses[i];
        geometry_msgs::msg::Pose pose_a = transformPoseFromCToA(pose_c, g_gamma);
        transformed_poses.poses.push_back(pose_a);
    }

    g_transformed_poses_pub->publish(transformed_poses);

    RCLCPP_DEBUG(g_node->get_logger(), "Inverse transformed %zu poses from C-frame to A-frame",
                 g_latest_poses->poses.size());
}

// 选中位姿回调函数 - 缓存 /pnp_poses_select（C系，单 pose）
void selectedPosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_latest_selected_poses = msg;
    RCLCPP_DEBUG(g_node->get_logger(), "Received /pnp_poses_select with %zu poses", msg->poses.size());
}

// Service callback to compute target location with energy mechanism rotation
void computeTargetLocationService(
    const std::shared_ptr<energy_pose_transform::srv::ComputeTargetLocation::Request> request,
    std::shared_ptr<energy_pose_transform::srv::ComputeTargetLocation::Response> response)
{
    std::lock_guard<std::mutex> lock(g_data_mutex);

    // Check if we have received pose data from /pnp_poses_select
    if (!g_latest_selected_poses || g_latest_selected_poses->poses.empty()) {
        RCLCPP_WARN(g_node->get_logger(), "No selected pose data available from /pnp_poses_select");
        response->success = false;
        return;
    }

    // Use the selected pose (C-frame) from /pnp_poses_select
    const auto& pose_c = g_latest_selected_poses->poses[0];
    
    // Get delta_mu from request
    double delta_mu = request->delta_mu;

    // D' coordinates (fixed point representing energy mechanism target center)
    // Note: D_z is always 0 for rotation in the XY plane
    double D_x = 0.0;
    double D_y = 0.7;

    // Step 1: Rotate D by delta_mu around Z-axis to get D'
    // x' = x*cos(delta_mu) - y*sin(delta_mu)
    // y' = x*sin(delta_mu) + y*cos(delta_mu)
    double cos_delta_mu = std::cos(delta_mu);
    double sin_delta_mu = std::sin(delta_mu);
    
    double D_prime_x = D_x * cos_delta_mu - D_y * sin_delta_mu;
    double D_prime_y = D_x * sin_delta_mu + D_y * cos_delta_mu;

    // Step 2: Get P_A from inverse transformation
    geometry_msgs::msg::Pose pose_a = transformPoseFromCToA(pose_c, g_gamma);
    
    // Step 3: Extract quaternion q_A from pose_a to get rotation matrix R_A
    double q_w = pose_a.orientation.w;
    double q_x = pose_a.orientation.x;
    double q_y = pose_a.orientation.y;
    double q_z = pose_a.orientation.z;

    // Compute rotation matrix R_A from quaternion q_A
    // R_A = [r11 r12 r13]
    //       [r21 r22 r23]
    //       [r31 r32 r33]
    // We only need r11, r12, r21, r22, r31, r32 for the final calculation
    double r11 = 1.0 - 2.0 * (q_y * q_y + q_z * q_z);
    double r12 = 2.0 * (q_x * q_y - q_w * q_z);
    
    double r21 = 2.0 * (q_x * q_y + q_w * q_z);
    double r22 = 1.0 - 2.0 * (q_x * q_x + q_z * q_z);
    
    double r31 = 2.0 * (q_x * q_z - q_w * q_y);
    double r32 = 2.0 * (q_y * q_z + q_w * q_x);

    // Step 4: Apply final transformation: P_final = R_A * D' + P_A
    // P_final = [r11*x' + r12*y' + x_A]
    //           [r21*x' + r22*y' + y_A]
    //           [r31*x' + r32*y' + z_A]
    geometry_msgs::msg::Point p_final;
    p_final.x = r11 * D_prime_x + r12 * D_prime_y + pose_a.position.x;
    p_final.y = r21 * D_prime_x + r22 * D_prime_y + pose_a.position.y;
    p_final.z = r31 * D_prime_x + r32 * D_prime_y + pose_a.position.z;

    // Set response
    response->p_final = p_final;
    response->success = true;

    // Publish to /target_location topic
    g_target_location_pub->publish(p_final);

    RCLCPP_INFO(g_node->get_logger(), 
                "Computed target location: [%.3f, %.3f, %.3f] with delta_mu=%.3f rad",
                p_final.x, p_final.y, p_final.z, delta_mu);
}

int main(int argc, char** argv)
{
    // 1. 初始化 ROS2
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("coordinate_c2a_node");

    // 2. 从配置文件读取逆变换的平移参数
    try {
        std::string package_share_dir = ament_index_cpp::get_package_share_directory("pose_transform");
        std::string config_file = package_share_dir + "/config/transform_params.yaml";
        
        RCLCPP_INFO(g_node->get_logger(), "Loading inverse transform config from: %s", config_file.c_str());
        
        if (loadInverseTranslationFromYAML(config_file, g_translation_x1_prime, g_translation_y1_prime, g_translation_z1_prime)) {
            RCLCPP_INFO(g_node->get_logger(), "Successfully loaded inverse translation parameters from config file");
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

    RCLCPP_INFO(g_node->get_logger(), "Coordinate C2A Inverse Transform Node started");
    RCLCPP_INFO(g_node->get_logger(), "Inverse Translation: x1_prime=%.3f, y1_prime=%.3f, z1_prime=%.3f",
                g_translation_x1_prime, g_translation_y1_prime, g_translation_z1_prime);
    RCLCPP_INFO(g_node->get_logger(), "Angles: theta=%.3f rad, gamma=%.3f rad", g_theta, g_gamma);

    // 4. 创建订阅者 - 订阅 /pnp_poses_ground (C系下的位姿)
    g_poses_sub = g_node->create_subscription<geometry_msgs::msg::PoseArray>(
        "/ene_pnp_poses_ground", rclcpp::SensorDataQoS(), posesCallback);

    // 订阅选中目标的C系位姿（来自 Pose_Transform_Node 变换后的 /pnp_poses_selected_a）
    g_selected_poses_sub = g_node->create_subscription<geometry_msgs::msg::PoseArray>(
        "/ene_pnp_poses_select", rclcpp::QoS(10), selectedPosesCallback);

    // 订阅关节状态，更新 pitch / yaw 角度（弧度）
    g_joint_state_sub = g_node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::QoS(10), jointStateCallback);

    // 5. 创建发布者 - 发布到 /pnp_poses_barrel (枪管坐标系A'下的位姿)
    g_transformed_poses_pub = g_node->create_publisher<geometry_msgs::msg::PoseArray>(
        "/ene_pnp_poses_barrel", rclcpp::SensorDataQoS());

    // 6. 创建发布者 - 发布到 /target_location (目标位置)
    g_target_location_pub = g_node->create_publisher<geometry_msgs::msg::Point>(
        "/ene_target_location", 10);

    // 7. 创建服务 - 计算目标位置
    g_compute_service = g_node->create_service<energy_pose_transform::srv::ComputeTargetLocation>(
        "compute_target_location",
        computeTargetLocationService);

    RCLCPP_INFO(g_node->get_logger(), "Subscribed to: /pnp_poses_ground");
    RCLCPP_INFO(g_node->get_logger(), "Subscribed to: /pnp_poses_select");
    RCLCPP_INFO(g_node->get_logger(), "Publishing to: /pnp_poses_barrel");
    RCLCPP_INFO(g_node->get_logger(), "Publishing to: /target_location");
    RCLCPP_INFO(g_node->get_logger(), "Service available: compute_target_location (uses /pnp_poses_select)");

    // 8. 运行节点
    rclcpp::spin(g_node);

    // 9. 清理资源
    rclcpp::shutdown();
    return 0;
}
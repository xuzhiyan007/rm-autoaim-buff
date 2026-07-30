#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include <cmath>
#include <limits>


struct roll_data {
    double roll_angle;       // cos(yaw * 5), used for /pnp_roll_angles
    double roll_angle_raw;   // cos(yaw),     used for /pnp_roll_angles_each
    double yaw;              // raw yaw angle, used for clustering
    int count;
};

// Persistent state for A/B slot clustering
struct TrackState {
    double last_yaw_A = 0.0;  // last yaw of slot A (for clustering distance)
    double last_yaw_B = 0.0;  // last yaw of slot B (for clustering distance)
    bool initialized = false;
    int stale_A = 0;    // frames since slot A was last updated
    int stale_B = 999;  // B starts as stale (never seen)
    // Target selection: age = consecutive active frames; resets on new identity
    int age_A = 0;
    int age_B = 0;
    bool was_A_active = false;  // was slot A active (non-NaN) last frame
    bool was_B_active = false;
};

// Angular difference in [0, pi], handles +/-pi wraparound
inline double angular_diff(double a, double b) {
    return std::abs(std::atan2(std::sin(a - b), std::cos(a - b)));
}

// Result of A/B slot clustering (Hungarian-style minimum angular distance assignment)
struct ClusterResult {
    double slot_A = std::numeric_limits<double>::quiet_NaN();        // cos(yaw) assigned to slot A
    double slot_B = std::numeric_limits<double>::quiet_NaN();        // cos(yaw) assigned to slot B
    double roll_angle_A = std::numeric_limits<double>::quiet_NaN();  // cos(yaw*5) assigned to slot A
    double roll_angle_B = std::numeric_limits<double>::quiet_NaN();  // cos(yaw*5) assigned to slot B
    int index_A = -1;  // original pose array index for slot A (-1 = unassigned)
    int index_B = -1;  // original pose array index for slot B (-1 = unassigned)
};

// Result of target selection (age-based oldest-target strategy)
struct SelectionResult {
    double selected_roll = std::numeric_limits<double>::quiet_NaN();      // cos(yaw) of selected target
    double selected_roll_5x = std::numeric_limits<double>::quiet_NaN();   // cos(yaw*5) of selected target
    int selected_index = -1;  // original pose array index of selected target (-1 = none)
};

// Helper function: Calculate rotation angle around transformed Z-axis
roll_data calculateZRotationFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    // Convert quaternion to rotation matrix
    double w = q.w, x = q.x, y = q.y, z = q.z;
    
    //看theory.md中的公式推导
    
    double up = 2 * (q.w * q.z - q.x * q.y); 
    double down = 1 - 2*(y*y + z*z);
    double yaw = std::atan2(up, down);
    roll_data z_rotation;
    z_rotation.yaw            = yaw;
    z_rotation.roll_angle     = std::cos(yaw * 5);   // for /pnp_roll_angles
    z_rotation.roll_angle_raw = std::cos(yaw);        // for /pnp_roll_angles_each
    constexpr double DIFF = 2.0 * M_PI / 5.0;
    z_rotation.count = std::floor(yaw / DIFF);
    return z_rotation;
}


// A/B slot clustering (Hungarian-style) based on yaw angular distance.
// Updates state (last_yaw_A/B, stale_A/B, initialized) and returns the slot assignment.
ClusterResult clusterSlotsAB(
    const std::vector<double>& roll_angles_raw,
    const std::vector<double>& roll_angles,
    const std::vector<double>& yaws,
    TrackState& state,
    double switch_threshold,
    const rclcpp::Logger& logger)
{
    ClusterResult result;

    if (roll_angles_raw.size() == 1) {
        double r   = roll_angles_raw[0];
        double yaw = yaws[0];
        state.stale_A++;
        state.stale_B++;
        if (!state.initialized) {
            state.last_yaw_A = yaw;
            state.last_yaw_B = yaw;
            state.initialized = true;
            state.stale_A = 0;
            state.stale_B = 999;
            result.slot_A = r;
            result.roll_angle_A = roll_angles[0];
            result.index_A = 0;
        } else {
            double dist_A = angular_diff(yaw, state.last_yaw_A);
            double dist_B = angular_diff(yaw, state.last_yaw_B);
            // Prefer the most recently active slot;
            // only switch if the other slot is significantly closer (in radians)
            bool assign_to_A;
            if (state.stale_A <= state.stale_B) {
                assign_to_A = (dist_A - dist_B < switch_threshold);
            } else {
                assign_to_A = (dist_B - dist_A > switch_threshold);
            }
            if (assign_to_A) {
                state.last_yaw_A = yaw;
                state.stale_A = 0;
                result.slot_A = r;
                result.roll_angle_A = roll_angles[0];
                result.index_A = 0;
                RCLCPP_INFO(logger, "Single detection -> slot A (stale_B=%d)", state.stale_B);
            } else {
                state.last_yaw_B = yaw;
                state.stale_B = 0;
                result.slot_B = r;
                result.roll_angle_B = roll_angles[0];
                result.index_B = 0;
                RCLCPP_INFO(logger, "Single detection -> slot B (stale_A=%d)", state.stale_A);
            }
        }
    } else if (roll_angles_raw.size() >= 2) {
        double r0   = roll_angles_raw[0];
        double r1   = roll_angles_raw[1];
        double yaw0 = yaws[0];
        double yaw1 = yaws[1];
        if (!state.initialized) {
            state.last_yaw_A = yaw0;
            state.last_yaw_B = yaw1;
            state.initialized = true;
            state.stale_A = 0;
            state.stale_B = 0;
            result.slot_A = r0; result.slot_B = r1;
            result.roll_angle_A = roll_angles[0]; result.roll_angle_B = roll_angles[1];
            result.index_A = 0; result.index_B = 1;
        } else {
            // Hungarian-style: pick assignment with minimum total angular distance
            double cost_AB = angular_diff(yaw0, state.last_yaw_A) + angular_diff(yaw1, state.last_yaw_B);
            double cost_BA = angular_diff(yaw0, state.last_yaw_B) + angular_diff(yaw1, state.last_yaw_A);
            if (cost_AB <= cost_BA) {
                result.slot_A = r0; result.slot_B = r1;
                result.roll_angle_A = roll_angles[0]; result.roll_angle_B = roll_angles[1];
                state.last_yaw_A = yaw0;
                state.last_yaw_B = yaw1;
                result.index_A = 0; result.index_B = 1;
            } else {
                result.slot_A = r1; result.slot_B = r0;
                result.roll_angle_A = roll_angles[1]; result.roll_angle_B = roll_angles[0];
                state.last_yaw_A = yaw1;
                state.last_yaw_B = yaw0;
                result.index_A = 1; result.index_B = 0;
            }
            state.stale_A = 0;
            state.stale_B = 0;
        }
    }

    return result;
}

// Target selection: updates age with identity check, then selects the oldest-appearing target.
// Returns the selected target's cos(yaw), cos(yaw*5), and original pose array index.
SelectionResult selectTarget(
    const ClusterResult& cluster,
    TrackState& state,
    double old_yaw_A,
    double old_yaw_B,
    double identity_threshold,
    const rclcpp::Logger& logger)
{
    double slot_A = cluster.slot_A;
    double slot_B = cluster.slot_B;
    double roll_angle_A = cluster.roll_angle_A;
    double roll_angle_B = cluster.roll_angle_B;
    int index_A = cluster.index_A;
    int index_B = cluster.index_B;

    // --- Age update with identity check ---
    // Slot A age
    if (std::isnan(slot_A)) {
        state.age_A = 0;
    } else if (!state.was_A_active ||
               angular_diff(state.last_yaw_A, old_yaw_A) > identity_threshold) {
        state.age_A = 1;  // new or replaced target
    } else {
        state.age_A++;
    }
    state.was_A_active = !std::isnan(slot_A);

    // Slot B age
    if (std::isnan(slot_B)) {
        state.age_B = 0;
    } else if (!state.was_B_active ||
               angular_diff(state.last_yaw_B, old_yaw_B) > identity_threshold) {
        state.age_B = 1;  // new or replaced target
    } else {
        state.age_B++;
    }
    state.was_B_active = !std::isnan(slot_B);

    // --- Select oldest-appearing target ---
    //   both present  -> higher age wins; tie (simultaneous appearance) -> slot A
    //   only one      -> that one
    SelectionResult result;
    if (!std::isnan(slot_A) && !std::isnan(slot_B)) {
        bool pick_A = (state.age_A >= state.age_B);
        result.selected_roll = pick_A ? slot_A : slot_B;
        result.selected_roll_5x = pick_A ? roll_angle_A : roll_angle_B;
        result.selected_index = pick_A ? index_A : index_B;
        RCLCPP_INFO(logger,
            "Both slots active: age_A=%d age_B=%d -> selected %s",
            state.age_A, state.age_B, pick_A ? "A" : "B");
    } else if (!std::isnan(slot_A)) {
        result.selected_roll = slot_A;
        result.selected_roll_5x = roll_angle_A;
        result.selected_index = index_A;
    } else if (!std::isnan(slot_B)) {
        result.selected_roll = slot_B;
        result.selected_roll_5x = roll_angle_B;
        result.selected_index = index_B;
    }

    return result;
}

int main(int argc, char * argv[]) {
    // 1. Initialize ROS2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("inverse_pnp_node");

    // 1.1 Declare clustering parameters
    node->declare_parameter("switch_threshold", 0.3);
    node->declare_parameter("identity_threshold", 0.6);

    // 这下面很多注释掉的角度发布者是当时在做大能量机关选靶的时候为了调试选靶子功能是否稳定做的，后续不用的话可删除。

    // 2. Create publisher for mean z_rotation
    //auto publisher = node->create_publisher<std_msgs::msg::Float64>("/ene_pnp_roll_angles", 10);
    // 带采集时间戳的并行话题，供 little_energy EKF 节点使用（x = roll_angle, header.stamp = 图像采集时刻）
    auto stamped_publisher = node->create_publisher<geometry_msgs::msg::Vector3Stamped>("/ene_pnp_roll_angles_stamped", 10);
    
    // 3.1 Create publisher for roll_angles_count list
    //auto count_publisher = node->create_publisher<std_msgs::msg::Int32MultiArray>("/ene_pnp_roll_angles_count", 10);
    
    // 3.2 Create publisher for per-pose roll angles (before averaging)
    //auto each_angle_publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("/ene_pnp_roll_angles_each", 10);

    // 3.3 Create publisher for selected (oldest-appearing) target roll angle
    // auto selected_publisher = node->create_publisher<std_msgs::msg::Float64>("/ene_pnp_roll_angles_selected", 10);

    // 3.4 Create publisher for selected pose in A-frame (single-element PoseArray)
    auto selected_pose_publisher = node->create_publisher<geometry_msgs::msg::PoseArray>("/ene_pnp_poses_selected_a", 10);

    // 3.5 Create publisher for selected target raw yaw angle (radians) -> big_energy_fitting
    //auto yaw_publisher = node->create_publisher<std_msgs::msg::Float64>("/ene_pnp_yaw", 10);

    // Persistent A/B slot state for clustering
    auto state = std::make_shared<TrackState>();

    // 4. Create subscriber to /pnp_poses
    auto subscription = node->create_subscription<geometry_msgs::msg::PoseArray>(
        "/ene_pnp_poses", 10,
        [=](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
            
            RCLCPP_INFO(node->get_logger(), "Received PnP poses message with %zu poses", msg->poses.size());
            
            if (msg->poses.empty()) {
                RCLCPP_WARN(node->get_logger(), "Received empty PnP poses message. Skipping.");
                return;
            }
            
            // Result container for roll angles
            std::vector<double> roll_angles;
            std::vector<double> roll_angles_raw;
            std::vector<double> yaws;
            std::vector<int> roll_angles_count;
            // Process each pose
            for (size_t i = 0; i < msg->poses.size(); ++i) {
                const auto& pose = msg->poses[i];
                
                // Calculate rotation angle around transformed Z-axis
                roll_data z_rotation = calculateZRotationFromQuaternion(pose.orientation);
                roll_angles.push_back(z_rotation.roll_angle);
                roll_angles_count.push_back(z_rotation.count);
                yaws.push_back(z_rotation.yaw);
                
                // Separately collect raw roll angles (no ×5) for /pnp_roll_angles_each
                roll_angles_raw.push_back(z_rotation.roll_angle_raw);
                
                RCLCPP_INFO(node->get_logger(),
                           "Pose %zu - Z-axis rotation: %.6f",
                           i,
                           z_rotation.roll_angle);
            }
            
            // --- A/B slot clustering (Hungarian-style) based on roll_angle_raw ---
            // Save old yaw values before clustering overwrites them (for identity check)
            double old_yaw_A = state->last_yaw_A;
            double old_yaw_B = state->last_yaw_B;

            const double SWITCH_THRESHOLD = node->get_parameter("switch_threshold").as_double();
            ClusterResult cluster = clusterSlotsAB(
                roll_angles_raw, roll_angles, yaws, *state, SWITCH_THRESHOLD, node->get_logger());

            // --- Target selection (age-based oldest-target strategy) ---
            const double IDENTITY_THRESHOLD = node->get_parameter("identity_threshold").as_double();
            SelectionResult selection = selectTarget(
                cluster, *state, old_yaw_A, old_yaw_B, IDENTITY_THRESHOLD, node->get_logger());

            int selected_index = selection.selected_index;
            double selected_roll_angle_5x = selection.selected_roll_5x;

            // Publish selected pose in A-frame to /pnp_poses_selected_a
            if (selected_index >= 0 && static_cast<size_t>(selected_index) < msg->poses.size()) {
                geometry_msgs::msg::PoseArray selected_pose_arr;
                selected_pose_arr.header = msg->header;
                selected_pose_arr.poses.push_back(msg->poses[selected_index]);
                selected_pose_publisher->publish(selected_pose_arr);
                RCLCPP_INFO(node->get_logger(), "Published selected pose (index=%d) to /pnp_poses_selected_a",
                            selected_index);
            }

            // 同步发布带采集时间戳的话题（透传上游图像采集时刻）
            auto stamped_msg = geometry_msgs::msg::Vector3Stamped();
            stamped_msg.header = msg->header;  // 继承图像采集时间戳
            stamped_msg.vector.x = selected_roll_angle_5x;
            stamped_publisher->publish(stamped_msg);
        }
    );
    
    RCLCPP_INFO(node->get_logger(), "Z-Axis Rotation Calculator Node Started. Subscribing to /pnp_poses, publishing mean z_rotation to /pnp_roll_angles");
    
    // 5. Spin
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

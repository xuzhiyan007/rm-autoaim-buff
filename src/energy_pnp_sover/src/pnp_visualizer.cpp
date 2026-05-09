#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class PnPVisualizer : public rclcpp::Node {
public:
    PnPVisualizer() : Node("pnp_visualizer") {
        // Declare parameter for fixed frame (default: use original frame_id)
        this->declare_parameter("use_fixed_frame", false);
        this->declare_parameter("fixed_frame_id", "map");
        
        use_fixed_frame_ = this->get_parameter("use_fixed_frame").as_bool();
        fixed_frame_id_ = this->get_parameter("fixed_frame_id").as_string();
        
        // Create subscriber for /pnp_poses
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/ene_pnp_poses", 10,
            std::bind(&PnPVisualizer::poseArrayCallback, this, std::placeholders::_1)
        );

        // Create subscriber for /pnp_poses_ground
        subscription_ground_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/ene_pnp_poses_ground", 10,
            std::bind(&PnPVisualizer::poseArrayGroundCallback, this, std::placeholders::_1)
        );

        // Create publisher for visualization markers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/ene_pnp_visualization", 10
        );

        // Create publisher for ground truth visualization markers
        marker_pub_ground_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/ene_pnp_visualization_ground", 10
        );

        if (use_fixed_frame_) {
            RCLCPP_INFO(this->get_logger(), "PnP Visualizer Node Started. Using fixed frame: %s", fixed_frame_id_.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "PnP Visualizer Node Started. Using original frame_id from messages");
        }
    }

private:
    void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty PoseArray");
            return;
        }

        visualization_msgs::msg::MarkerArray marker_array;
        
        // Use fixed frame if enabled, otherwise use original frame_id
        std_msgs::msg::Header header = msg->header;
        if (use_fixed_frame_) {
            header.frame_id = fixed_frame_id_;
        }

        for (size_t i = 0; i < msg->poses.size(); ++i) {
            const auto& pose = msg->poses[i];

            // Create coordinate frame axes for each pose
            createAxisMarkers(marker_array, pose, header, i);

            // Create text label with position information
            createTextMarker(marker_array, pose, header, i);

            // Create arrow showing the orientation
            createArrowMarker(marker_array, pose, header, i);
        }

        marker_pub_->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), "Published visualization for %zu poses", msg->poses.size());
    }

    void poseArrayGroundCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty PoseArray (ground truth)");
            return;
        }

        visualization_msgs::msg::MarkerArray marker_array;
        
        // Use fixed frame if enabled, otherwise use original frame_id
        std_msgs::msg::Header header = msg->header;
        if (use_fixed_frame_) {
            header.frame_id = fixed_frame_id_;
        }

        for (size_t i = 0; i < msg->poses.size(); ++i) {
            const auto& pose = msg->poses[i];

            // Create coordinate frame axes for each pose (using different colors for ground truth)
            createAxisMarkersGround(marker_array, pose, header, i);

            // Create text label with position information
            createTextMarkerGround(marker_array, pose, header, i);

            // Create arrow showing the orientation
            createArrowMarkerGround(marker_array, pose, header, i);
        }

        marker_pub_ground_->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), "Published ground truth visualization for %zu poses", msg->poses.size());
    }

    void createAxisMarkers(visualization_msgs::msg::MarkerArray& marker_array,
                          const geometry_msgs::msg::Pose& pose,
                          const std_msgs::msg::Header& header,
                          size_t idx) {
        // X-axis (Red)
        visualization_msgs::msg::Marker x_axis;
        x_axis.header = header;
        x_axis.ns = "axes";
        x_axis.id = idx * 10 + 0;
        x_axis.type = visualization_msgs::msg::Marker::ARROW;
        x_axis.action = visualization_msgs::msg::Marker::ADD;
        x_axis.pose = pose;
        x_axis.scale.x = 0.2;  // Length
        x_axis.scale.y = 0.02; // Shaft diameter
        x_axis.scale.z = 0.02; // Head diameter
        x_axis.color.r = 1.0;
        x_axis.color.g = 0.0;
        x_axis.color.b = 0.0;
        x_axis.color.a = 0.8;
        x_axis.lifetime = rclcpp::Duration::from_seconds(0.2);

        // Y-axis (Green)
        visualization_msgs::msg::Marker y_axis = x_axis;
        y_axis.id = idx * 10 + 1;
        
        // Rotate 90 degrees around Z to point in Y direction
        tf2::Quaternion q_pose, q_rot, q_new;
        tf2::fromMsg(pose.orientation, q_pose);
        q_rot.setRPY(0, 0, M_PI/2);
        q_new = q_pose * q_rot;
        y_axis.pose.orientation = tf2::toMsg(q_new);
        
        y_axis.color.r = 0.0;
        y_axis.color.g = 1.0;
        y_axis.color.b = 0.0;

        // Z-axis (Blue)
        visualization_msgs::msg::Marker z_axis = x_axis;
        z_axis.id = idx * 10 + 2;
        
        // Rotate -90 degrees around Y to point in Z direction
        q_rot.setRPY(0, -M_PI/2, 0);
        q_new = q_pose * q_rot;
        z_axis.pose.orientation = tf2::toMsg(q_new);
        
        z_axis.color.r = 0.0;
        z_axis.color.g = 0.0;
        z_axis.color.b = 1.0;

        marker_array.markers.push_back(x_axis);
        marker_array.markers.push_back(y_axis);
        marker_array.markers.push_back(z_axis);
    }

    void createTextMarker(visualization_msgs::msg::MarkerArray& marker_array,
                         const geometry_msgs::msg::Pose& pose,
                         const std_msgs::msg::Header& header,
                         size_t idx) {
        visualization_msgs::msg::Marker text;
        text.header = header;
        text.ns = "labels";
        text.id = idx * 10 + 3;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        
        text.pose.position.x = pose.position.x;
        text.pose.position.y = pose.position.y;
        text.pose.position.z = pose.position.z + 0.15; // Offset above the pose
        text.pose.orientation.w = 1.0;
        
        text.scale.z = 0.08; // Text height
        text.color.r = 1.0;
        text.color.g = 1.0;
        text.color.b = 1.0;
        text.color.a = 1.0;
        
        // Format: Target #idx: (x, y, z)
        char buffer[100];
        snprintf(buffer, sizeof(buffer), "Target #%zu\n(%.2f, %.2f, %.2f)m",
                idx, pose.position.x, pose.position.y, pose.position.z);
        text.text = buffer;
        
        text.lifetime = rclcpp::Duration::from_seconds(0.2);

        marker_array.markers.push_back(text);
    }

    void createArrowMarker(visualization_msgs::msg::MarkerArray& marker_array,
                          const geometry_msgs::msg::Pose& pose,
                          const std_msgs::msg::Header& header,
                          size_t idx) {
        visualization_msgs::msg::Marker arrow;
        arrow.header = header;
        arrow.ns = "direction";
        arrow.id = idx * 10 + 4;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.pose = pose;
        
        arrow.scale.x = 0.3;  // Length
        arrow.scale.y = 0.03; // Shaft diameter
        arrow.scale.z = 0.05; // Head diameter
        arrow.color.r = 1.0;
        arrow.color.g = 1.0;
        arrow.color.b = 0.0;
        arrow.color.a = 0.6;
        arrow.lifetime = rclcpp::Duration::from_seconds(0.2);

        marker_array.markers.push_back(arrow);
    }

    void createAxisMarkersGround(visualization_msgs::msg::MarkerArray& marker_array,
                          const geometry_msgs::msg::Pose& pose,
                          const std_msgs::msg::Header& header,
                          size_t idx) {
        // X-axis (Darker Red for ground truth)
        visualization_msgs::msg::Marker x_axis;
        x_axis.header = header;
        x_axis.ns = "axes_ground";
        x_axis.id = idx * 10 + 0;
        x_axis.type = visualization_msgs::msg::Marker::ARROW;
        x_axis.action = visualization_msgs::msg::Marker::ADD;
        x_axis.pose = pose;
        x_axis.scale.x = 0.2;  // Length
        x_axis.scale.y = 0.02; // Shaft diameter
        x_axis.scale.z = 0.02; // Head diameter
        x_axis.color.r = 0.7;
        x_axis.color.g = 0.0;
        x_axis.color.b = 0.0;
        x_axis.color.a = 0.8;
        x_axis.lifetime = rclcpp::Duration::from_seconds(0.2);

        // Y-axis (Darker Green for ground truth)
        visualization_msgs::msg::Marker y_axis = x_axis;
        y_axis.id = idx * 10 + 1;
        
        // Rotate 90 degrees around Z to point in Y direction
        tf2::Quaternion q_pose, q_rot, q_new;
        tf2::fromMsg(pose.orientation, q_pose);
        q_rot.setRPY(0, 0, M_PI/2);
        q_new = q_pose * q_rot;
        y_axis.pose.orientation = tf2::toMsg(q_new);
        
        y_axis.color.r = 0.0;
        y_axis.color.g = 0.7;
        y_axis.color.b = 0.0;

        // Z-axis (Darker Blue for ground truth)
        visualization_msgs::msg::Marker z_axis = x_axis;
        z_axis.id = idx * 10 + 2;
        
        // Rotate -90 degrees around Y to point in Z direction
        q_rot.setRPY(0, -M_PI/2, 0);
        q_new = q_pose * q_rot;
        z_axis.pose.orientation = tf2::toMsg(q_new);
        
        z_axis.color.r = 0.0;
        z_axis.color.g = 0.0;
        z_axis.color.b = 0.7;

        marker_array.markers.push_back(x_axis);
        marker_array.markers.push_back(y_axis);
        marker_array.markers.push_back(z_axis);
    }

    void createTextMarkerGround(visualization_msgs::msg::MarkerArray& marker_array,
                         const geometry_msgs::msg::Pose& pose,
                         const std_msgs::msg::Header& header,
                         size_t idx) {
        visualization_msgs::msg::Marker text;
        text.header = header;
        text.ns = "labels_ground";
        text.id = idx * 10 + 3;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        
        text.pose.position.x = pose.position.x;
        text.pose.position.y = pose.position.y;
        text.pose.position.z = pose.position.z + 0.15; // Offset above the pose
        text.pose.orientation.w = 1.0;
        
        text.scale.z = 0.08; // Text height
        text.color.r = 0.0;
        text.color.g = 1.0;
        text.color.b = 1.0;
        text.color.a = 1.0;
        
        // Format: Ground #idx: (x, y, z)
        char buffer[100];
        snprintf(buffer, sizeof(buffer), "Ground #%zu\n(%.2f, %.2f, %.2f)m",
                idx, pose.position.x, pose.position.y, pose.position.z);
        text.text = buffer;
        
        text.lifetime = rclcpp::Duration::from_seconds(0.2);

        marker_array.markers.push_back(text);
    }

    void createArrowMarkerGround(visualization_msgs::msg::MarkerArray& marker_array,
                          const geometry_msgs::msg::Pose& pose,
                          const std_msgs::msg::Header& header,
                          size_t idx) {
        visualization_msgs::msg::Marker arrow;
        arrow.header = header;
        arrow.ns = "direction_ground";
        arrow.id = idx * 10 + 4;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.pose = pose;
        
        arrow.scale.x = 0.3;  // Length
        arrow.scale.y = 0.03; // Shaft diameter
        arrow.scale.z = 0.05; // Head diameter
        arrow.color.r = 0.0;
        arrow.color.g = 1.0;
        arrow.color.b = 1.0;
        arrow.color.a = 0.6;
        arrow.lifetime = rclcpp::Duration::from_seconds(0.2);

        marker_array.markers.push_back(arrow);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_ground_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_ground_;
    bool use_fixed_frame_;
    std::string fixed_frame_id_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PnPVisualizer>());
    rclcpp::shutdown();
    return 0;
}

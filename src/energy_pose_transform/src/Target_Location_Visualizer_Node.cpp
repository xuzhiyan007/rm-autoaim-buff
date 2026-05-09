#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <memory>

class TargetLocationVisualizer : public rclcpp::Node
{
public:
    TargetLocationVisualizer()
    : Node("target_location_visualizer")
    {
        // Declare parameters
        this->declare_parameter<std::string>("frame_id", "camera_frame");
        this->declare_parameter<double>("marker_scale", 0.1);
        this->declare_parameter<double>("marker_alpha", 1.0);
        this->declare_parameter<double>("marker_lifetime", 0.5);
        
        // Get parameters
        frame_id_ = this->get_parameter("frame_id").as_string();
        marker_scale_ = this->get_parameter("marker_scale").as_double();
        marker_alpha_ = this->get_parameter("marker_alpha").as_double();
        marker_lifetime_ = this->get_parameter("marker_lifetime").as_double();
        
        // Create subscriber for /target_location
        target_location_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/ene_target_location",
            10,
            std::bind(&TargetLocationVisualizer::targetLocationCallback, this, std::placeholders::_1)
        );
        
        // Create publisher for visualization marker
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/ene_target_location_marker",
            10
        );
        
        RCLCPP_INFO(this->get_logger(), "Target Location Visualizer Node started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /target_location");
        RCLCPP_INFO(this->get_logger(), "Publishing markers to: /target_location_marker");
        RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frame_id_.c_str());
    }

private:
    void targetLocationCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        // Create a sphere marker for the target location
        visualization_msgs::msg::Marker marker;
        
        // Header
        marker.header.frame_id = frame_id_;
        marker.header.stamp = this->now();
        
        // Marker properties
        marker.ns = "target_location";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Position from the received point
        marker.pose.position.x = msg->x;
        marker.pose.position.y = msg->y;
        // marker.pose.position.z = msg->z;
        marker.pose.position.z = msg->z;
        
        // Orientation (default)
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        
        // Scale
        marker.scale.x = marker_scale_;
        marker.scale.y = marker_scale_;
        marker.scale.z = marker_scale_;
        
        // Color (red)
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = marker_alpha_;
        
        // Lifetime
        marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
        
        // Publish the marker
        marker_pub_->publish(marker);
        
        // Also create a text marker to show coordinates
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = frame_id_;
        text_marker.header.stamp = this->now();
        text_marker.ns = "target_location_text";
        text_marker.id = 1;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Position slightly above the point
        text_marker.pose.position.x = msg->x;
        text_marker.pose.position.y = msg->y;
        text_marker.pose.position.z = 0 + 0.15;
        
        text_marker.pose.orientation.w = 1.0;
        
        // Text scale
        text_marker.scale.z = 0.08;
        
        // Color (white)
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        
        // Text content
        char text_buffer[100];
        snprintf(text_buffer, sizeof(text_buffer), "[%.2f, %.2f, %.2f]", msg->x, msg->y, msg->z);
        text_marker.text = text_buffer;
        
        text_marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
        
        // Publish text marker
        marker_pub_->publish(text_marker);
        
        RCLCPP_DEBUG(this->get_logger(), "Visualized target location: [%.3f, %.3f, %.3f]",
                     msg->x, msg->y, msg->z);
    }
    
    // Member variables
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_location_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    
    std::string frame_id_;
    double marker_scale_;
    double marker_alpha_;
    double marker_lifetime_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetLocationVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

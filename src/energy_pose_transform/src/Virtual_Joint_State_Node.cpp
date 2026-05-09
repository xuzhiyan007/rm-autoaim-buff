#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <string>

namespace
{
constexpr int kTrackbarMax = 180;  // represents range [-90deg, 90deg]
int g_pitch_trackbar = kTrackbarMax / 2;
int g_yaw_trackbar = kTrackbarMax / 2;

inline double degToRad(double deg)
{
  const double pi = 3.14159265358979323846;
  return deg * pi / 180.0;
}

inline double trackbarToRad(int value)
{
  // map [0, kTrackbarMax] -> [-90deg, 90deg]
  double deg = static_cast<double>(value - kTrackbarMax / 2);
  return degToRad(deg);
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("virtual_joint_state_node");

  // timestamp offset parameter, consistent with jlcv_serial_driver
  double timestamp_offset = node->declare_parameter("timestamp_offset", 0.0);

  auto joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::QoS(rclcpp::KeepLast(1)));

  const std::string window_name = "Virtual Joints";
  cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
  cv::createTrackbar("pitch (deg)", window_name, &g_pitch_trackbar, kTrackbarMax);
  cv::createTrackbar("yaw (deg)", window_name, &g_yaw_trackbar, kTrackbarMax);

  rclcpp::WallRate rate(50.0);  // 50 Hz publish rate

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    sensor_msgs::msg::JointState joint_state;
    timestamp_offset = node->get_parameter("timestamp_offset").as_double();
    joint_state.header.stamp = node->now() + rclcpp::Duration::from_seconds(timestamp_offset);

    double pitch = trackbarToRad(g_pitch_trackbar);
    double yaw = trackbarToRad(g_yaw_trackbar);

    joint_state.name.push_back("pitch_joint");
    joint_state.position.push_back(pitch);
    joint_state.name.push_back("yaw_joint");
    joint_state.position.push_back(yaw);

    joint_state_pub->publish(joint_state);

    // simple visualization of current values
    cv::Mat display(100, 480, CV_8UC3, cv::Scalar(50, 50, 50));
    std::string text = "pitch: " + std::to_string(pitch) + " rad, yaw: " + std::to_string(yaw) + " rad";
    cv::putText(display, text, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6,
      cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::imshow(window_name, display);

    int key = cv::waitKey(1);
    if (key == 27) {  // ESC to quit
      break;
    }

    rate.sleep();
  }

  cv::destroyWindow(window_name);
  rclcpp::shutdown();
  return 0;
}

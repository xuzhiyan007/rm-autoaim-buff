#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <limits>

class VisualizerNode : public rclcpp::Node
{
public:
    VisualizerNode() : Node("visualizer_node")
    {
        // Declare parameters
        this->declare_parameter("image_topic", "/image_raw");
        this->declare_parameter("keypoints_topic", "/ene_detection_keypoints");
        this->declare_parameter("window_name", "Detection Visualization");

        // Get parameters
        image_topic_ = this->get_parameter("image_topic").as_string();
        keypoints_topic_ = this->get_parameter("keypoints_topic").as_string();
        window_name_ = this->get_parameter("window_name").as_string();

        // Create QoS profile with BestEffort reliability to match the publisher
        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);

        // Create message filter subscribers with explicit QoS
        image_sub_.subscribe(this, image_topic_, qos_profile);
        keypoints_sub_.subscribe(this, keypoints_topic_, qos_profile);

        // Create synchronizer with approximate time policy
        sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), image_sub_, keypoints_sub_);
        sync_->registerCallback(std::bind(&VisualizerNode::sync_callback, this,
                                          std::placeholders::_1, std::placeholders::_2));

        // Initialize keypoint colors (BGR format for OpenCV)
        // Matches colors from onnx_infer_node visualization
        kp_colors_ = {
            cv::Scalar(0, 0, 255),    // Red   - keypoint 0 (R center)
            cv::Scalar(255, 0, 0),    // Blue  - keypoint 1
            cv::Scalar(0, 255, 255),  // Yellow - keypoint 2
            cv::Scalar(255, 0, 255),  // Magenta - keypoint 3
            cv::Scalar(0, 255, 255),  // Cyan  - keypoint 4
            cv::Scalar(128, 0, 128)   // Purple - keypoint 5
        };

        RCLCPP_INFO(this->get_logger(), "Visualizer Node initialized");
        RCLCPP_INFO(this->get_logger(), "Image topic: %s", image_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Keypoints topic: %s", keypoints_topic_.c_str());
    }

    ~VisualizerNode()
    {
        cv::destroyAllWindows();
    }

private:
    void sync_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                       const geometry_msgs::msg::PoseArray::ConstSharedPtr& keypoints_msg)
    {
        try {
            // 零拷贝：直接将消息数据指针包装为 cv::Mat，不分配新内存
            const cv::Mat image(
                static_cast<int>(image_msg->height),
                static_cast<int>(image_msg->width),
                CV_8UC3,
                const_cast<uint8_t*>(image_msg->data.data()),
                static_cast<size_t>(image_msg->step));
            if (image.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Received empty image");
                return;
            }

            // Clone 用于绘制（可写副本，不修改原始消息数据）
            // /image_raw 为 RGB 编码，转换为 OpenCV 期望的 BGR 格式
            cv::Mat vis_image;
            cv::cvtColor(image.clone(), vis_image, cv::COLOR_RGB2BGR);

            // ── Parse PoseArray ──────────────────────────────────────────────────
            // Protocol from openvino_infer_node:
            //   z = -1 : separator – start of a new detection object
            //   z >= 0 : keypoint  – (x, y) pixel coords, z = output index
            //   z = -2 : circle ROI top-left  (x, y)
            //   z = -3 : circle ROI bottom-right (x, y)
            //   z = -4 : R mark ROI top-left  (x, y)
            //   z = -5 : R mark ROI bottom-right (x, y)
            struct DetObj {
                std::vector<cv::Point2f> keypoints;
                cv::Point2f circle_tl, circle_br;
                cv::Point2f r_tl,      r_br;
                bool has_circle_roi = false;
                bool has_r_roi      = false;
            };

            std::vector<DetObj> all_objects;
            DetObj cur;
            bool in_object = false;

            for (const auto& pose : keypoints_msg->poses) {
                float z = static_cast<float>(pose.position.z);
                float px = static_cast<float>(pose.position.x);
                float py = static_cast<float>(pose.position.y);

                if (z == -1.0f) {
                    if (in_object) all_objects.push_back(cur);
                    cur = DetObj{};
                    in_object = true;
                } else if (z == -2.0f) {
                    cur.circle_tl = cv::Point2f(px, py);
                } else if (z == -3.0f) {
                    cur.circle_br = cv::Point2f(px, py);
                    cur.has_circle_roi = true;
                } else if (z == -4.0f) {
                    cur.r_tl = cv::Point2f(px, py);
                } else if (z == -5.0f) {
                    cur.r_br = cv::Point2f(px, py);
                    cur.has_r_roi = true;
                } else {
                    cur.keypoints.push_back(cv::Point2f(px, py));
                }
            }
            if (in_object) all_objects.push_back(cur);

            // ── Draw each detection ──────────────────────────────────────────────
            for (const auto& obj : all_objects) {
                const auto& kps = obj.keypoints;

                // Circle ROI – cyan rectangle
                if (obj.has_circle_roi) {
                    cv::rectangle(vis_image,
                                  cv::Point((int)obj.circle_tl.x, (int)obj.circle_tl.y),
                                  cv::Point((int)obj.circle_br.x, (int)obj.circle_br.y),
                                  cv::Scalar(255, 255, 0), 2);
                    cv::putText(vis_image, "Circle ROI",
                               cv::Point((int)obj.circle_tl.x + 2, (int)obj.circle_tl.y - 5),
                               cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);
                }

                // R mark ROI – orange rectangle
                if (obj.has_r_roi) {
                    cv::rectangle(vis_image,
                                  cv::Point((int)obj.r_tl.x, (int)obj.r_tl.y),
                                  cv::Point((int)obj.r_br.x, (int)obj.r_br.y),
                                  cv::Scalar(0, 128, 255), 2);
                    cv::putText(vis_image, "R ROI",
                               cv::Point((int)obj.r_tl.x + 2, (int)obj.r_tl.y - 5),
                               cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 128, 255), 1);
                }

                // Green bounding box from keypoints 1-4
                if (kps.size() >= 5) {
                    float min_x = std::numeric_limits<float>::max();
                    float min_y = std::numeric_limits<float>::max();
                    float max_x = std::numeric_limits<float>::lowest();
                    float max_y = std::numeric_limits<float>::lowest();
                    for (size_t i = 1; i < kps.size() && i <= 4; ++i) {
                        min_x = std::min(min_x, kps[i].x);
                        min_y = std::min(min_y, kps[i].y);
                        max_x = std::max(max_x, kps[i].x);
                        max_y = std::max(max_y, kps[i].y);
                    }
                    cv::rectangle(vis_image,
                                  cv::Point((int)min_x, (int)min_y),
                                  cv::Point((int)max_x, (int)max_y),
                                  cv::Scalar(0, 255, 0), 2);
                }

                // Draw each keypoint with its index
                for (size_t k = 0; k < kps.size(); ++k) {
                    cv::Point center((int)kps[k].x, (int)kps[k].y);
                    cv::Scalar color = kp_colors_[k % kp_colors_.size()];
                    cv::circle(vis_image, center, 5, color, -1);
                    cv::putText(vis_image, std::to_string(k),
                               cv::Point(center.x + 8, center.y - 8),
                               cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
                }
            }

            // Display visualization window
            cv::imshow(window_name_, vis_image);
            cv::waitKey(1);

            RCLCPP_DEBUG(this->get_logger(), "Visualized %zu objects", all_objects.size());

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Visualization exception: %s", e.what());
        }
    }

    // Message filter types
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        geometry_msgs::msg::PoseArray>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

    // Subscribers
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<geometry_msgs::msg::PoseArray> keypoints_sub_;
    std::shared_ptr<Synchronizer> sync_;

    // Parameters
    std::string image_topic_;
    std::string keypoints_topic_;
    std::string window_name_;

    // Visualization colors
    std::vector<cv::Scalar> kp_colors_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisualizerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

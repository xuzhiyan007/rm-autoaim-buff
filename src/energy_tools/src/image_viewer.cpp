#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <csignal>
#include <chrono>
#include <thread>

// 全局变量（遵循现有模式）
rclcpp::Node::SharedPtr g_node = nullptr;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr g_image_sub = nullptr;
bool g_should_exit = false;

// 信号处理函数
void SignalHandler(int signum) {
    if (g_node) {
        RCLCPP_INFO(g_node->get_logger(), "Received signal %d, shutting down...", signum);
    }
    g_should_exit = true;
}

// 图像回调函数（零拷贝：直接将消息数据包装为 cv::Mat，无内存拷贝）
void RawImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        // 零拷贝包装：直接引用消息内部数据指针，不分配新内存
        cv::Mat image(
            static_cast<int>(msg->height),
            static_cast<int>(msg->width),
            CV_8UC3,
            const_cast<uint8_t*>(msg->data.data()),
            static_cast<size_t>(msg->step));

        if (image.empty()) {
            RCLCPP_WARN(g_node->get_logger(), "Received empty image");
            return;
        }

        // 调整图像大小至宽度为1024，保持宽高比
        cv::Mat resized_image;
        int target_width = 1024;
        int original_width = image.cols;
        int original_height = image.rows;

        if (original_width != target_width) {
            int target_height = static_cast<int>(
                static_cast<double>(original_height) * target_width / original_width);
            cv::resize(image, resized_image, cv::Size(target_width, target_height),
                       0, 0, cv::INTER_LINEAR);
        } else {
            // 已是目标宽度，clone 一份用于显示（避免持有消息内存引用到下一帧）
            resized_image = image.clone();
        }

        // RGB -> BGR 转换（hik_camera 发布 rgb8，OpenCV imshow 期望 BGR）
        cv::cvtColor(resized_image, resized_image, cv::COLOR_RGB2BGR);

        // 显示图像
        cv::imshow("Raw Image Feed", resized_image);
        cv::waitKey(1);

        // 每5秒输出一次统计信息
        static int frame_count = 0;
        static auto last_stats_time = std::chrono::steady_clock::now();
        frame_count++;

        auto current_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(
            current_time - last_stats_time);

        if (duration.count() >= 5) {
            double fps = static_cast<double>(frame_count) / duration.count();
            RCLCPP_INFO(g_node->get_logger(),
                       "Stats: %.1f FPS, Frame %d, Raw size: %zu bytes, "
                       "Original: %dx%d -> Resized: %dx%d",
                       fps, frame_count, msg->data.size(),
                       original_width, original_height,
                       resized_image.cols, resized_image.rows);
            frame_count = 0;
            last_stats_time = current_time;
        }

    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(g_node->get_logger(), "OpenCV error: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(g_node->get_logger(), "Error processing image: %s", e.what());
    }
}

int main(int argc, char** argv) {
    // 初始化ROS2
    rclcpp::init(argc, argv);

    // 创建节点
    g_node = rclcpp::Node::make_shared("image_viewer");

    // 设置信号处理
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    // 配置QoS（与 hik_camera_node 保持一致：sensor_data profile，BestEffort + Volatile）
    rclcpp::QoS qos_profile(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);

    // 声明话题名称参数（与 hik_camera_node 发布话题 /image_raw 对齐）
    std::string topic_name = g_node->declare_parameter("topic_name", "/image_raw");

    g_image_sub = g_node->create_subscription<sensor_msgs::msg::Image>(
        topic_name, qos_profile, RawImageCallback);

    // 创建OpenCV窗口
    cv::namedWindow("Raw Image Feed", cv::WINDOW_AUTOSIZE);
    cv::startWindowThread();

    RCLCPP_INFO(g_node->get_logger(),
               "Raw image viewer started (zero-copy). Listening to: %s", topic_name.c_str());
    RCLCPP_INFO(g_node->get_logger(), "Press Ctrl+C, Esc, or Q to exit");

    // ROS2 spin
    while (rclcpp::ok() && !g_should_exit) {
        rclcpp::spin_some(g_node);

        // 检测按键退出（Esc=27, Q=113, q=81）
        int key = cv::waitKey(1);
        if (key == 27 || key == 113 || key == 81) {
            RCLCPP_INFO(g_node->get_logger(), "User requested exit via key press");
            g_should_exit = true;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 清理
    RCLCPP_INFO(g_node->get_logger(), "Shutting down raw image viewer...");
    cv::destroyAllWindows();
    rclcpp::shutdown();

    return 0;
}

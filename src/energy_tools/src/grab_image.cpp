#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <csignal>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <cstring>
#include <ctime>

rclcpp::Node::SharedPtr g_node = nullptr;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr g_image_sub = nullptr;
bool g_should_exit = false;

std::string g_size_type = "little";
std::string g_color_type = "red";
std::string g_group = "01";
int g_save_every_n_frames = 8;

std::string getCurrentDateTimeStr() {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_info;
    localtime_r(&t, &tm_info);
    std::ostringstream oss;
    oss << std::put_time(&tm_info, "%Y%m%d_%H%M");
    return oss.str();
}

void getUserInput() {
    std::cout << "\n=== Image Saver Configuration ===" << std::endl;
    std::string input;

    std::cout << "Select size type:" << std::endl;
    std::cout << "  1. little" << std::endl;
    std::cout << "  2. big" << std::endl;
    std::cout << "Enter choice (1/2, default: 1-little): ";
    std::getline(std::cin, input);
    g_size_type = (input == "2") ? "big" : "little";

    std::cout << "Select color type:" << std::endl;
    std::cout << "  1. red" << std::endl;
    std::cout << "  2. blue" << std::endl;
    std::cout << "Enter choice (1/2, default: 1-red): ";
    std::getline(std::cin, input);
    g_color_type = (input == "2") ? "blue" : "red";

    std::cout << "Enter group number (e.g. 01, default: " << g_group << "): ";
    std::getline(std::cin, input);
    if (!input.empty()) {
        g_group = input;
    }

    std::cout << "Save every N frames (default: " << g_save_every_n_frames << "): ";
    std::getline(std::cin, input);
    if (!input.empty()) {
        try {
            int val = std::stoi(input);
            if (val > 0) {
                g_save_every_n_frames = val;
            } else {
                std::cout << "Invalid value, using default: " << g_save_every_n_frames << std::endl;
            }
        } catch (...) {
            std::cout << "Invalid value, using default: " << g_save_every_n_frames << std::endl;
        }
    }

    std::cout << "\nFinal configuration: " << g_size_type << "_" << g_color_type << "_" << g_group
              << ", save every " << g_save_every_n_frames << " frames" << std::endl;
    std::cout << "=================================" << std::endl << std::endl;
}

void parseArguments(int argc, char** argv) {
    bool has_args = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--size") == 0 && i + 1 < argc) {
            g_size_type = argv[++i];
            has_args = true;
        } else if (strcmp(argv[i], "--color") == 0 && i + 1 < argc) {
            g_color_type = argv[++i];
            has_args = true;
        } else if (strcmp(argv[i], "--group") == 0 && i + 1 < argc) {
            g_group = argv[++i];
            has_args = true;
        } else if (strcmp(argv[i], "--every") == 0 && i + 1 < argc) {
            g_save_every_n_frames = std::atoi(argv[++i]);
            has_args = true;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --size <little|big>    Set size type (default: little)\n"
                      << "  --color <red|blue>     Set color type (default: red)\n"
                      << "  --group <group>        Set group number (default: 01)\n"
                      << "  --every <N>            Save every N frames (default: 8)\n"
                      << "  --help, -h             Show this help message\n"
                      << "\nIf no arguments provided, interactive mode will be used." << std::endl;
            exit(0);
        }
    }

    if (!has_args) {
        getUserInput();
    }
}

void SignalHandler(int signum) {
    if (g_node) {
        RCLCPP_INFO(g_node->get_logger(), "Received signal %d, shutting down...", signum);
    }
    g_should_exit = true;
}

class ImageSaver : public rclcpp::Node {
public:
    ImageSaver() : Node("image_saver") {
        topic_name_ = this->declare_parameter("topic_name", "/image_raw");
        image_format_ = this->declare_parameter("image_format", "jpg");
        save_every_n_frames_ = this->declare_parameter("save_every_n_frames", g_save_every_n_frames);

        // 节点启动时固定时间戳（精确到分钟），保证同一次采集会话内命名前缀一致
        std::string datetime_str = getCurrentDateTimeStr();
        std::string base_name = datetime_str + "_" + g_size_type + "_" + g_color_type + "_" + g_group;
        save_directory_ = base_name;
        filename_prefix_ = base_name;

        sequence_number_ = 1;
        frame_count_ = 0;

        ensureDirectoryExists(save_directory_);

        std::string ts_path = save_directory_ + "/timestamps.txt";
        timestamps_file_.open(ts_path, std::ios::app);
        if (!timestamps_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open timestamps file: %s", ts_path.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Timestamps file: %s", ts_path.c_str());
        }

        auto qos_profile = rclcpp::SensorDataQoS();
        qos_profile.keep_last(5);
        qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        g_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            topic_name_, qos_profile,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                this->ImageCallback(msg);
            });

        RCLCPP_INFO(this->get_logger(), "Image saver node started");
        RCLCPP_INFO(this->get_logger(), "Listening to topic: %s", topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "Save directory: %s", save_directory_.c_str());
        RCLCPP_INFO(this->get_logger(), "Image format: %s", image_format_.c_str());
        RCLCPP_INFO(this->get_logger(), "Save every N frames: %d", save_every_n_frames_);
    }

private:
    void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            frame_count_++;

            if (frame_count_ % save_every_n_frames_ != 0) {
                return;
            }

            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");

            if (cv_ptr->image.empty()) {
                RCLCPP_WARN(this->get_logger(), "Received empty image");
                return;
            }

            // 文件名格式：[日期时间]_[size]_[color]_[组别]_[序号(4位补零)].[格式]
            std::stringstream filename;
            filename << filename_prefix_ << "_"
                     << std::setw(4) << std::setfill('0') << sequence_number_
                     << "." << image_format_;
            std::string full_path = save_directory_ + "/" + filename.str();

            bool success = cv::imwrite(full_path, cv_ptr->image);

            if (success) {
                size_t image_size = msg->step * msg->height;
                RCLCPP_INFO(this->get_logger(), "Saved image: %s (size: %dx%d, bytes: %zu)",
                           filename.str().c_str(), cv_ptr->image.cols, cv_ptr->image.rows, image_size);

                if (timestamps_file_.is_open()) {
                    timestamps_file_ << filename.str() << " "
                                     << msg->header.stamp.sec << " "
                                     << msg->header.stamp.nanosec << "\n";
                    timestamps_file_.flush();
                }

                sequence_number_++;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to save image: %s", filename.str().c_str());
            }

            if (sequence_number_ % 10 == 1) {
                static auto last_stats_time = std::chrono::steady_clock::now();
                auto current_time = std::chrono::steady_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_stats_time);

                if (duration.count() > 0) {
                    double save_rate = 10.0 / duration.count();
                    RCLCPP_INFO(this->get_logger(), "Stats: Saved %d images, rate: %.1f fps",
                               sequence_number_ - 1, save_rate);
                }
                last_stats_time = current_time;
            }

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV error: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing image: %s", e.what());
        }
    }

    bool ensureDirectoryExists(const std::string& path) {
        try {
            std::filesystem::create_directories(path);
            RCLCPP_INFO(this->get_logger(), "Ensured save directory exists: %s", path.c_str());
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create directory: %s, error: %s", path.c_str(), e.what());
            return false;
        }
    }

    std::string topic_name_;
    std::string save_directory_;
    std::string image_format_;
    std::string filename_prefix_;
    int save_every_n_frames_;

    int frame_count_;
    int sequence_number_;

    std::ofstream timestamps_file_;
};

int main(int argc, char** argv) {
    parseArguments(argc, argv);

    rclcpp::init(argc, argv);

    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    g_node = std::make_shared<ImageSaver>();

    while (rclcpp::ok() && !g_should_exit) {
        rclcpp::spin_some(g_node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    RCLCPP_INFO(g_node->get_logger(), "Shutting down image saver node...");
    rclcpp::shutdown();

    return 0;
}

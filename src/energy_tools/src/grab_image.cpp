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

// Global variables (following existing pattern)
rclcpp::Node::SharedPtr g_node = nullptr;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr g_image_sub = nullptr;
bool g_should_exit = false;

// Command line arguments
std::string g_save_directory = "angle1_blue_min_sim1";
std::string g_filename_prefix = "01_0_0_1_";

// Function to get user input from terminal
void getUserInput() {
    std::cout << "\n=== Image Saver Configuration ===" << std::endl;
    std::cout << "Current settings:" << std::endl;
    std::cout << "  Save directory: " << g_save_directory << std::endl;
    std::cout << "  Filename prefix: " << g_filename_prefix << std::endl;
    std::cout << std::endl;

    std::cout << "Would you like to change these settings? (y/n): ";
    std::string response;
    std::getline(std::cin, response);

    if (response == "y" || response == "Y" || response == "yes" || response == "YES") {
        // Get save directory
        std::cout << "Enter save directory name (default: " << g_save_directory << "): ";
        std::string dir_input;
        std::getline(std::cin, dir_input);
        if (!dir_input.empty()) {
            g_save_directory = dir_input;
        }

        // Get filename prefix
        std::cout << "Enter filename prefix (default: " << g_filename_prefix << "): ";
        std::string prefix_input;
        std::getline(std::cin, prefix_input);
        if (!prefix_input.empty()) {
            g_filename_prefix = prefix_input;
        }
    }

    std::cout << "\nFinal configuration:" << std::endl;
    std::cout << "  Save directory: " << g_save_directory << std::endl;
    std::cout << "  Filename prefix: " << g_filename_prefix << std::endl;
    std::cout << "=================================" << std::endl;
    std::cout << std::endl;
}

// Function to parse command line arguments
void parseArguments(int argc, char** argv) {
    bool has_dir_arg = false;
    bool has_prefix_arg = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--dir") == 0 && i + 1 < argc) {
            g_save_directory = argv[++i];
            has_dir_arg = true;
        } else if (strcmp(argv[i], "--prefix") == 0 && i + 1 < argc) {
            g_filename_prefix = argv[++i];
            has_prefix_arg = true;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --dir <directory>     Set save directory (default: angle1_blue_min_sim1)\n"
                      << "  --prefix <prefix>     Set filename prefix (default: 01_0_0_1_)\n"
                      << "  --help, -h            Show this help message\n"
                      << "\nIf no command line arguments are provided, the program will prompt"
                      << "\nfor input during runtime." << std::endl;
            exit(0);
        }
    }

    // If no command line arguments were provided, get user input interactively
    if (!has_dir_arg && !has_prefix_arg) {
        getUserInput();
    }
}

// Signal handler
void SignalHandler(int signum) {
    if (g_node) {
        RCLCPP_INFO(g_node->get_logger(), "Received signal %d, shutting down...", signum);
    }
    g_should_exit = true;
}

class ImageSaver : public rclcpp::Node {
public:
    ImageSaver() : Node("image_saver") {
        // Declare configuration parameters
        topic_name_ = this->declare_parameter("topic_name", "/image_raw");
        save_directory_ = this->declare_parameter("save_directory", g_save_directory);
        image_format_ = this->declare_parameter("image_format", "jpg");
        save_every_n_frames_ = this->declare_parameter("save_every_n_frames", 1);
        starting_sequence_number_ = this->declare_parameter("starting_sequence_number", 1);

        // Store filename prefix from command line
        filename_prefix_ = g_filename_prefix;

        // Handle tilde in directory path
        if (save_directory_[0] == '~') {
            const char* home = getenv("HOME");
            if (home) {
                save_directory_ = std::string(home) + save_directory_.substr(1);
            }
        }

        // Initialize sequence number
        sequence_number_ = starting_sequence_number_;
        frame_count_ = 0;

        // Ensure save directory exists
        ensureDirectoryExists(save_directory_);

        // Open timestamps file (append mode)
        std::string ts_path = save_directory_ + "/timestamps.txt";
        timestamps_file_.open(ts_path, std::ios::app);
        if (!timestamps_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open timestamps file: %s", ts_path.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Timestamps file: %s", ts_path.c_str());
        }

        // Create subscriber (with optimized QoS)
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
        RCLCPP_INFO(this->get_logger(), "Starting sequence number: %d", starting_sequence_number_);
        RCLCPP_INFO(this->get_logger(), "Save every N frames: %d", save_every_n_frames_);
    }

private:
    void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            frame_count_++;

            // Check if this frame should be saved
            if (frame_count_ % save_every_n_frames_ != 0) {
                return;
            }

            // Convert ROS image message to OpenCV Mat using cv_bridge
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            
            if (cv_ptr->image.empty()) {
                RCLCPP_WARN(this->get_logger(), "Received empty image");
                return;
            }

            // Generate filename (6-digit fixed format)
            std::stringstream filename;
            filename << filename_prefix_ << "_" << std::setw(6) << std::setfill('0') << sequence_number_ << "." << image_format_;
            std::string full_path = save_directory_ + "/" + filename.str();

            // Save image
            bool success = cv::imwrite(full_path, cv_ptr->image);

            if (success) {
                size_t image_size = msg->step * msg->height;
                RCLCPP_INFO(this->get_logger(), "Saved image: %s (size: %dx%d, bytes: %zu)",
                           filename.str().c_str(), cv_ptr->image.cols, cv_ptr->image.rows, image_size);

                // Write timestamp: filename sec nanosec
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

            // Statistics (every 10 frames)
            if (sequence_number_ % 10 == 1) {
                static auto last_stats_time = std::chrono::steady_clock::now();
                auto current_time = std::chrono::steady_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_stats_time);

                if (duration.count() > 0) {
                    double save_rate = 10.0 / duration.count();
                    RCLCPP_INFO(this->get_logger(), "Stats: Saved %d images, rate: %.1f fps",
                               sequence_number_ - starting_sequence_number_, save_rate);
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

    // Configuration parameters
    std::string topic_name_;
    std::string save_directory_;
    std::string image_format_;
    std::string filename_prefix_;
    int save_every_n_frames_;
    int starting_sequence_number_;

    // State tracking
    int frame_count_;
    int sequence_number_;

    // Timestamps file
    std::ofstream timestamps_file_;
};

int main(int argc, char** argv) {
    // Parse command line arguments first (before ROS2 initializes)
    parseArguments(argc, argv);

    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Set signal handling
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    // Create image saver node
    g_node = std::make_shared<ImageSaver>();

    // ROS2 spin
    while (rclcpp::ok() && !g_should_exit) {
        rclcpp::spin_some(g_node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Cleanup
    RCLCPP_INFO(g_node->get_logger(), "Shutting down image saver node...");
    rclcpp::shutdown();

    return 0;
}
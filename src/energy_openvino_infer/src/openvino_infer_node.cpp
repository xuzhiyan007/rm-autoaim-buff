#include <openvino/openvino.hpp>
#include <openvino/core/preprocess/pre_post_process.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <string>
#include <filesystem>
#include <chrono>
#include <algorithm>
#include <cstdlib>
#include <pwd.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#if __has_include(<cv_bridge/cv_bridge.h>)
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif
#include <future>  // for std::async
#include <yaml-cpp/yaml.h>
#include "rclcpp_components/register_node_macro.hpp"


// Default — template matching ON, R refine ON, visualization OFF
//ros2 run openvino_infer openvino_infer_node --ros-args -p enable_visualization:=true

//Disable template matching
//ros2 run openvino_infer openvino_infer_node --ros-args -p enable_template_matching:=false

//Disable R refine
//ros2 run openvino_infer openvino_infer_node --ros-args -p enable_r_refine:=false

//Combine with visualization
//ros2 run openvino_infer openvino_infer_node --ros-args -p enable_template_matching:=false -p enable_r_refine:=false -p enable_visualization:=true

struct Keypoint {
    float x;
    float y;
    float visibility;
};

struct TemplateData {
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::Point2f> specific_pts;
    int min_matches;
    float ratio_test;
    cv::Ptr<cv::DescriptorMatcher> matcher; // 使用基类类型，支持 FLANN 和 BFMatcher
};

struct SiftParams {
    int nfeatures = 0;
    int nOctaveLayers = 3;
    double contrastThreshold = 0.01;
    double edgeThreshold = 30.0;
    double sigma = 1.6;
};

struct TemplateConfig {
    std::vector<cv::Point2f> specific_pts;
    SiftParams sift;
    int min_matches = 4;
    float ratio_test = 0.7f;
};

struct HomographyConfig {
    float ransac_threshold = 5.0f;
    float margin_ratio = 0.1f;
};

class OpenVINOInferenceNode : public rclcpp::Node
{
public:
    explicit OpenVINOInferenceNode(const rclcpp::NodeOptions & options)
        : rclcpp::Node("openvino_infer_node", options)
    {
        try {
            initialize_ros2();      // 先初始化ROS2参数，获取 model_path_
            initialize_openvino();  // 然后再加载OpenVINO模型
            RCLCPP_INFO(this->get_logger(), "OpenVINO Inference Node initialized successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize: %s", e.what());
            throw;
        }
    }

    ~OpenVINOInferenceNode()
    {
        if (enable_visualization_) {
            cv::destroyAllWindows();
        }
    }

private:
    void initialize_ros2()
    {
#ifdef USE_YOLO26
        RCLCPP_INFO(this->get_logger(), "=== YOLO26 (NMS-Free) mode enabled ===");
#else
        RCLCPP_INFO(this->get_logger(), "=== YOLO12 mode enabled ===");
#endif
        // 声明参数 - 使用 ~ 表示用户主目录，更具可移植性
#ifdef USE_YOLO26
        this->declare_parameter("model_path", "~/infantry/src/energy_openvino_infer/models26/best26.onnx");
#else
        this->declare_parameter("model_path", "~/infantry/src/energy_openvino_infer/models/best.xml");
#endif
        this->declare_parameter("template_circle_path", "~/infantry/src/energy_openvino_infer/template/template_circle.png");
        this->declare_parameter("input_topic", "/image_raw");
        this->declare_parameter("output_topic", "/detections");
        this->declare_parameter("keypoints_topic", "/ene_detection_keypoints");
        this->declare_parameter("confidence_threshold", 0.5);
        this->declare_parameter("nms_threshold", 0.4);
        this->declare_parameter("input_width", 640);
        this->declare_parameter("input_height", 640);
        this->declare_parameter("device", "GPU");
        this->declare_parameter("num_threads", 4);
        this->declare_parameter("class_names", std::vector<std::string>{});
        this->declare_parameter("enable_visualization", true);
        this->declare_parameter("enable_template_matching", true);
        this->declare_parameter("performance_mode", std::string("latency"));  // latency / throughput / cumulative_throughput  低延时/高吞吐/积累吞吐
        this->declare_parameter("template_circle_scale", 0.5);
        this->declare_parameter("template_circle_roi_scale", 0.5);
        this->declare_parameter("template_channel_mode", "red"); //gray, red,  blue
        this->declare_parameter("kp0_hold_max_pixel_dist", 30.0);  // kp[0]补帧阈値：圆形关键点平均偏移上限（像素）
        this->declare_parameter("enable_kp0_hold", true);            // 是否开启 kp[0] 补帧策略
        this->declare_parameter("enable_mode_gate", false);           // 是否启用 /robot_mode 门控
        this->declare_parameter("roi_expand_ratio", 8.0 / 3.0);     // R/Circle ROI 扩展倍数（base_box → expanded_roi）
        this->declare_parameter("r_roi_base_ratio", 3.0 / 5.0);     // R base box 相对圆形包围盒（c_base_box）的宽高比例
        this->declare_parameter("r_detect_threshold", 27);            // R 红色通道二值化阈值
        this->declare_parameter("r_detect_radius_ratio", 1.1);        // R Mask 半径比例（相对 kp4-kp2 距离）
        this->declare_parameter("r_detect_extend_ratio", 2.4);        // R 圆心粗估计延伸倍数
        this->declare_parameter("r_detect_dilate_kernel", 5);         // R 膨胀核大小
        this->declare_parameter("enable_r_refine", true);              // 是否开启 kp[0] 红色通道精检测修正
        this->declare_parameter("r_refine_max_dev_ratio", 0.5);        // kp[0] 精检测偏差保护比例（相对检测框对角线）

        // 获取参数并展开路径
        model_path_ = expandPath(this->get_parameter("model_path").as_string());
        std::string template_circle_path = expandPath(this->get_parameter("template_circle_path").as_string());
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        keypoints_topic_ = this->get_parameter("keypoints_topic").as_string();
        confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();
        nms_threshold_ = this->get_parameter("nms_threshold").as_double();
        input_width_ = this->get_parameter("input_width").as_int();
        input_height_ = this->get_parameter("input_height").as_int();
        device_ = this->get_parameter("device").as_string();
        num_threads_ = this->get_parameter("num_threads").as_int();
        class_names_ = this->get_parameter("class_names").as_string_array();
        enable_visualization_ = this->get_parameter("enable_visualization").as_bool();
        enable_template_matching_ = this->get_parameter("enable_template_matching").as_bool();
        performance_mode_ = this->get_parameter("performance_mode").as_string();
        template_circle_scale_ = static_cast<float>(this->get_parameter("template_circle_scale").as_double());
        template_circle_roi_scale_ = static_cast<float>(this->get_parameter("template_circle_roi_scale").as_double());
        template_channel_mode_ = this->get_parameter("template_channel_mode").as_string();
        kp0_hold_max_pixel_dist_ = static_cast<float>(this->get_parameter("kp0_hold_max_pixel_dist").as_double());
        enable_kp0_hold_ = this->get_parameter("enable_kp0_hold").as_bool();
        enable_mode_gate_ = this->get_parameter("enable_mode_gate").as_bool();
        roi_expand_ratio_ = static_cast<float>(this->get_parameter("roi_expand_ratio").as_double());
        r_roi_base_ratio_ = static_cast<float>(this->get_parameter("r_roi_base_ratio").as_double());
        r_detect_threshold_ = this->get_parameter("r_detect_threshold").as_int();
        r_detect_radius_ratio_ = static_cast<float>(this->get_parameter("r_detect_radius_ratio").as_double());
        r_detect_extend_ratio_ = static_cast<float>(this->get_parameter("r_detect_extend_ratio").as_double());
        r_detect_dilate_kernel_ = this->get_parameter("r_detect_dilate_kernel").as_int();
        enable_r_refine_ = this->get_parameter("enable_r_refine").as_bool();
        r_refine_max_dev_ratio_ = static_cast<float>(this->get_parameter("r_refine_max_dev_ratio").as_double());
        this->declare_parameter("sift_params_path", "~/infantry/src/energy_openvino_infer/config/SIFT_params.yaml");
        std::string sift_params_path = expandPath(this->get_parameter("sift_params_path").as_string());
        load_sift_params(sift_params_path);

        // Circle 模板初始化专用 SIFT
        auto sift_c_tmpl = cv::SIFT::create(
            circle_cfg_.sift.nfeatures,
            circle_cfg_.sift.nOctaveLayers,
            circle_cfg_.sift.contrastThreshold,
            circle_cfg_.sift.edgeThreshold,
            circle_cfg_.sift.sigma
        );

        RCLCPP_INFO(this->get_logger(), "Circle template path: %s", template_circle_path.c_str());

        cv::Mat img_c = cv::imread(template_circle_path, cv::IMREAD_COLOR);
        if (!img_c.empty()) {
            cv::cvtColor(img_c, img_c, cv::COLOR_BGR2RGB);  // imread 返回 BGR，统一转为 RGB 与运行时一致
            cv::Mat img_c_channel;
            extract_channel(img_c, img_c_channel);
            cv::Mat img_c_small;
            cv::resize(img_c_channel, img_c_small, cv::Size(), template_circle_scale_, template_circle_scale_, cv::INTER_LINEAR);
            sift_c_tmpl->detectAndCompute(img_c_small, cv::noArray(), circle_td_.keypoints, circle_td_.descriptors);
            std::vector<cv::Point2f> c_pts_scaled;
            for (const auto& pt : circle_cfg_.specific_pts) {
                c_pts_scaled.emplace_back(pt.x * template_circle_scale_, pt.y * template_circle_scale_);
            }
            circle_td_.specific_pts = std::move(c_pts_scaled);
            circle_td_.min_matches = circle_cfg_.min_matches;
            circle_td_.ratio_test = circle_cfg_.ratio_test;
            // 优化：使用 BFMatcher 替代 FLANN（ROI特征点少时更快）
            circle_td_.matcher = cv::makePtr<cv::BFMatcher>(cv::NORM_L2, false);
            circle_td_.matcher->add({circle_td_.descriptors});
            // BFMatcher 无需 train()，直接使用
            RCLCPP_INFO(this->get_logger(), "Circle template loaded and BFMatcher initialized successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to read template_circle.png from: %s", template_circle_path.c_str());
        }

        auto qos_profile = rclcpp::SensorDataQoS();
        qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic_, qos_profile,
            std::bind(&OpenVINOInferenceNode::image_callback, this, std::placeholders::_1));
        // 注意：回调签名为 UniquePtr，与发布端 publish(std::move) 配合实现进程内零拷贝

        // /robot_mode 门控订阅：非 ENERGY 模式时跳过推理
        mode_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/robot_mode", rclcpp::QoS(rclcpp::KeepLast(1)),
            [this](const std_msgs::msg::UInt8::SharedPtr msg) {
                robot_mode_.store(msg->data);
            });

        keypoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            keypoints_topic_, qos_profile);

        RCLCPP_INFO(this->get_logger(), "Model path: %s", model_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Keypoints topic: %s", keypoints_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Template matching: %s", enable_template_matching_ ? "enabled" : "disabled");
    }

    // 路径展开函数：支持 ~、环境变量、相对路径
    std::string expandPath(const std::string& path)
    {
        std::string expanded_path = path;

        // 1. 展开波浪号 (~) 为用户主目录
        if (!expanded_path.empty() && expanded_path[0] == '~') {
            const char* home = std::getenv("HOME");
            if (!home) {
                // 如果 HOME 环境变量不存在，使用 getpwuid 获取
                struct passwd* pw = getpwuid(getuid());
                home = pw ? pw->pw_dir : nullptr;
            }
            if (home) {
                expanded_path = std::string(home) + expanded_path.substr(1);
                RCLCPP_DEBUG(this->get_logger(), "Expanded ~ to: %s", expanded_path.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Could not expand ~ in path: %s", path.c_str());
            }
        }

        // 2. 展开环境变量 (例如: $HOME/path 或 ${HOME}/path)
        size_t pos = 0;
        while ((pos = expanded_path.find('$', pos)) != std::string::npos) {
            size_t end_pos;
            size_t var_start;
            
            if (pos + 1 < expanded_path.length() && expanded_path[pos + 1] == '{') {
                // ${VAR} 格式
                var_start = pos + 2;
                end_pos = expanded_path.find('}', var_start);
                if (end_pos == std::string::npos) break;
            } else {
                // $VAR 格式
                var_start = pos + 1;
                end_pos = var_start;
                while (end_pos < expanded_path.length() && 
                       (std::isalnum(expanded_path[end_pos]) || expanded_path[end_pos] == '_')) {
                    end_pos++;
                }
            }
            
            std::string var_name = expanded_path.substr(var_start, end_pos - var_start);
            const char* var_value = std::getenv(var_name.c_str());
            
            if (var_value) {
                size_t replace_end = (pos + 1 < expanded_path.length() && expanded_path[pos + 1] == '{') ? end_pos + 1 : end_pos;
                expanded_path.replace(pos, replace_end - pos, var_value);
                pos += std::strlen(var_value);
                RCLCPP_DEBUG(this->get_logger(), "Expanded $%s to: %s", var_name.c_str(), var_value);
            } else {
                RCLCPP_WARN(this->get_logger(), "Environment variable not found: %s", var_name.c_str());
                pos++;
            }
        }

        // 3. 转换为绝对路径
        try {
            std::filesystem::path fs_path(expanded_path);
            
            // 如果是相对路径，转换为绝对路径
            if (fs_path.is_relative()) {
                fs_path = std::filesystem::absolute(fs_path);
            }
            
            expanded_path = fs_path.string();
            
            // 4. 验证路径是否存在
            if (!std::filesystem::exists(expanded_path)) {
                RCLCPP_WARN(this->get_logger(), "Path does not exist: %s", expanded_path.c_str());
                RCLCPP_WARN(this->get_logger(), "Original path was: %s", path.c_str());
            } else {
                RCLCPP_DEBUG(this->get_logger(), "Resolved path: %s", expanded_path.c_str());
            }
        } catch (const std::filesystem::filesystem_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Filesystem error: %s", e.what());
        }

        return expanded_path;
    }

    void initialize_openvino()
    {
        try {
            std::string model_path = model_path_;
            if (!std::filesystem::exists(model_path)) {
                throw std::runtime_error("Model file not found: " + model_path);
            }
            
            RCLCPP_INFO(this->get_logger(), "Loading model from: %s", model_path.c_str());
            
            std::shared_ptr<ov::Model> model = core.read_model(model_path);
            
            std::vector<std::string> available_devices = core.get_available_devices();
            std::string requested_device = device_;
            if (requested_device.empty()) {
                requested_device = "GPU";
            }
            
            // ===== OpenVINO 预处理优化：将颜色转换、归一化、NCHW 转置嵌入模型图 =====
            {
                ov::preprocess::PrePostProcessor ppp(model);
                ppp.input().tensor()
                    .set_element_type(ov::element::u8)                    // 原始输入: uint8
                    .set_layout("NHWC")                                    // OpenCV 格式 H x W x C
                    .set_color_format(ov::preprocess::ColorFormat::RGB);  // 输入图像为 RGB 格式

                ppp.input().preprocess()
                    .convert_element_type(ov::element::f32)               // 1) uint8 -> float32
                    .scale(255.0f)                                        // 2) /255.0
                    .convert_layout("NCHW");                              // 3) HWC -> NCHW

                ppp.input().model().set_layout("NCHW");
                model = ppp.build();
            }
            // ===================================================================
            
            bool gpu_attempted = false;
            bool gpu_succeeded = false;
            std::string fallback_reason;
            std::string target_device = requested_device;
            
            if (requested_device == "GPU") {
                gpu_attempted = true;
                bool gpu_available = std::find(available_devices.begin(), available_devices.end(), "GPU") != available_devices.end();
                if (!gpu_available) {
                    fallback_reason = "GPU device not available";
                    RCLCPP_WARN(this->get_logger(), "GPU device not found! Available devices:");
                    for (const auto& d : available_devices) {
                        RCLCPP_WARN(this->get_logger(), "  - %s", d.c_str());
                    }
                } else {
                    // 设置 GPU 推理配置（可选优化）
                    //core.set_property("GPU", ov::hint::inference_precision(ov::element::f16));

                    ov::hint::PerformanceMode perf_mode = ov::hint::PerformanceMode::LATENCY;
                    if (performance_mode_ == "throughput") {
                        perf_mode = ov::hint::PerformanceMode::THROUGHPUT;
                    } else if (performance_mode_ == "cumulative_throughput") {
                        perf_mode = ov::hint::PerformanceMode::CUMULATIVE_THROUGHPUT;
                    }
                    core.set_property("GPU", ov::hint::performance_mode(perf_mode));
                    RCLCPP_INFO(this->get_logger(), "GPU performance mode: %s", performance_mode_.c_str());
                    
                    try {
                        RCLCPP_INFO(this->get_logger(), "Attempting to compile model on GPU...");
                        compiled_model = core.compile_model(model, "GPU");
                        gpu_succeeded = true;
                    } catch (const std::exception& e) {
                        fallback_reason = std::string("GPU compile failed: ") + e.what();
                        RCLCPP_WARN(this->get_logger(), "GPU compile failed: %s", e.what());
                    }
                }
            } else if (requested_device == "CPU") {
                core.set_property("CPU", ov::inference_num_threads(num_threads_));
                RCLCPP_INFO(this->get_logger(), "CPU inference threads: %d", num_threads_);
                compiled_model = core.compile_model(model, "CPU");
            } else {
                // 其他设备（如 AUTO, MULTI 等），直接尝试
                compiled_model = core.compile_model(model, requested_device);
            }
            
            // GPU 尝试失败，回退到 CPU
            if (gpu_attempted && !gpu_succeeded) {
                target_device = "CPU";
                RCLCPP_WARN(this->get_logger(), "===============================================");
                RCLCPP_WARN(this->get_logger(), " Falling back to CPU: %s", fallback_reason.c_str());
                RCLCPP_WARN(this->get_logger(), "===============================================");
                
                core.set_property("CPU", ov::inference_num_threads(num_threads_));
                RCLCPP_INFO(this->get_logger(), "CPU inference threads: %d", num_threads_);
                compiled_model = core.compile_model(model, "CPU");
            }
            
            if (gpu_succeeded || (!gpu_attempted && target_device != "CPU")) {
                RCLCPP_INFO(this->get_logger(), "===============================================");
                RCLCPP_INFO(this->get_logger(), " Using %s for inference", target_device.c_str());
                RCLCPP_INFO(this->get_logger(), "===============================================");
            }
            
            RCLCPP_INFO(this->get_logger(), "Using device: %s", target_device.c_str());
            
            infer_request = compiled_model.create_infer_request();
            
            input_name = compiled_model.input().get_any_name();
            output_name = compiled_model.output().get_any_name();
            
            // 预分配OpenCV图像缓冲区
            letterboxed_image_.create(input_height_, input_width_, CV_8UC3);
            resized_image_.create(input_height_, input_width_, CV_8UC3);
            
            RCLCPP_INFO(this->get_logger(), "Model loaded and compiled successfully on %s", target_device.c_str());
            RCLCPP_INFO(this->get_logger(), "Input name: %s", input_name.c_str());
            RCLCPP_INFO(this->get_logger(), "Output name: %s", output_name.c_str());
            RCLCPP_INFO(this->get_logger(), "Pre-allocated buffers: %dx%d", input_width_, input_height_);
            
        } catch (const std::exception& e) {
            throw std::runtime_error("Failed to initialize OpenVINO: " + std::string(e.what()));
        }
    }

    void image_callback(sensor_msgs::msg::Image::UniquePtr msg)
    {
        uint8_t mode = robot_mode_.load();
        if (enable_mode_gate_ && mode != 0 && mode != 1) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "[openvino_infer] standby (mode=%d, waiting for SMALL_ENERGY=0 or BIG_ENERGY=1)",
                static_cast<int>(mode));
            return;
        }
        try {
            auto t_start = std::chrono::high_resolution_clock::now();

            // UniquePtr 独占所有权，clone() 拷贝出独立 cv::Mat 供后续异步处理，
            // 同时避免直接操作消息缓冲区带来的潜在风险。
            cv::Mat image = cv::Mat(
                static_cast<int>(msg->height),
                static_cast<int>(msg->width),
                CV_8UC3,
                msg->data.data(),
                static_cast<size_t>(msg->step)).clone();
            if (image.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Empty image received");
                return;
            }
            auto t_convert = std::chrono::high_resolution_clock::now();

            std::vector<cv::Rect> boxes;
            std::vector<float> confidences;
            std::vector<int> class_ids;
            std::vector<std::vector<Keypoint>> all_keypoints;
            
            run_inference(image, boxes, confidences, class_ids, all_keypoints);
            auto t_infer = std::chrono::high_resolution_clock::now();

            std::vector<int> indices;
#ifdef USE_YOLO26
            // NMS-Free: model output is already top-K (300), just collect all
            indices.reserve(confidences.size());
            for (size_t i = 0; i < confidences.size(); i++) {
                indices.push_back(static_cast<int>(i));
            }
#else
            cv::dnn::NMSBoxes(boxes, confidences, confidence_threshold_, nms_threshold_, indices);
#endif
            auto t_nms = std::chrono::high_resolution_clock::now();

            if (indices.size() > 0) {
                geometry_msgs::msg::PoseArray keypoints_msg;
                keypoints_msg.header.stamp = msg->header.stamp;  //直接复用相机的时间戳
                keypoints_msg.header.frame_id = "camera_frame";

                for (int idx : indices) {
                    if (idx >= all_keypoints.size()) continue;

                    cv::Rect box = boxes[idx] & cv::Rect(0, 0, image.cols, image.rows);

                    cv::Rect c_expanded_roi; bool has_c_roi = false;

                    // 辅助 lambda：将 base_box 扩大后提取灰度并按指定 scale 缩放
                    auto expand_and_extract = [&](const cv::Rect& base_box, cv::Mat& out_roi_gray_small, cv::Rect& out_expanded_roi, float roi_scale) -> bool {
                        int expand_width = (int)(base_box.width * roi_expand_ratio_);
                        int expand_height = (int)(base_box.height * roi_expand_ratio_);
                        int expand_x = base_box.x - (expand_width - base_box.width) / 2;
                        int expand_y = base_box.y - (expand_height - base_box.height) / 2;

                        expand_x = std::max(0, std::min(expand_x, image.cols - expand_width));
                        expand_y = std::max(0, std::min(expand_y, image.rows - expand_height));
                        expand_width = std::min(expand_width, image.cols - expand_x);
                        expand_height = std::min(expand_height, image.rows - expand_y);

                        if (expand_width <= 0 || expand_height <= 0) return false;
                        out_expanded_roi = cv::Rect(expand_x, expand_y, expand_width, expand_height);

                        cv::Mat roi_channel;
                        extract_channel(image(out_expanded_roi), roi_channel);
                        cv::resize(roi_channel, out_roi_gray_small, cv::Size(), roi_scale, roi_scale, cv::INTER_LINEAR);
                        return true;
                    };

                    // 先计算关键点 2-5 的包围盒（供 Circle ROI 共用）
                    cv::Rect c_base_box;
                    bool has_c_base = false;
                    if (all_keypoints[idx].size() >= 6) {
                        float min_x = static_cast<float>(image.cols);
                        float max_x = 0.0f;
                        float min_y = static_cast<float>(image.rows);
                        float max_y = 0.0f;
                        for (int k = 2; k <= 5; ++k) {
                            if (all_keypoints[idx][k].visibility > 0.5f) {
                                min_x = std::min(min_x, all_keypoints[idx][k].x);
                                max_x = std::max(max_x, all_keypoints[idx][k].x);
                                min_y = std::min(min_y, all_keypoints[idx][k].y);
                                max_y = std::max(max_y, all_keypoints[idx][k].y);
                                has_c_base = true;
                            }
                        }
                        if (has_c_base) {
                            c_base_box = cv::Rect((int)min_x, (int)min_y, (int)(max_x - min_x), (int)(max_y - min_y));
                            c_base_box &= cv::Rect(0, 0, image.cols, image.rows);
                        }
                    }

                    // 结构体：存储匹配结果
                    struct MatchResult {
                        bool success = false;
                        bool has_roi = false;
                        cv::Rect expanded_roi;
                        std::vector<cv::Point2f> points;
                    };

                    // 并行处理 R 和 Circle 模板匹配
                    // 在模板匹配前记录神经网络原始 kp[0] 检测状态，供补帧逻辑使用
                    const bool nn_kp0_detected = (all_keypoints[idx].size() > 0 &&
                                                  all_keypoints[idx][0].visibility > 0.5f);
                    if (enable_template_matching_) {
                    auto c_future = std::async(std::launch::async, [&]() -> MatchResult {
                        MatchResult res;
                        if (has_c_base && c_base_box.width > 0 && c_base_box.height > 0) {
                            cv::Mat c_roi_gray_small;
                            if (expand_and_extract(c_base_box, c_roi_gray_small, res.expanded_roi, template_circle_roi_scale_)) {
                                res.has_roi = true;
                                thread_local cv::Ptr<cv::SIFT> tl_sift_c = cv::SIFT::create(
                                    roi_sift_cfg_.nfeatures,
                                    roi_sift_cfg_.nOctaveLayers,
                                    roi_sift_cfg_.contrastThreshold,
                                    roi_sift_cfg_.edgeThreshold,
                                    roi_sift_cfg_.sigma);
                                if (match_template_local(c_roi_gray_small, circle_td_, res.points, tl_sift_c)) {
                                    res.success = true;
                                }
                            }
                        }
                        return res;
                    });

                    // 等待任务完成并应用结果
                    MatchResult c_result = c_future.get();

                    // 应用 Circle 模板结果
                    if (c_result.has_roi) {
                        has_c_roi = true;
                        c_expanded_roi = c_result.expanded_roi;
                        if (c_result.success) {
                            float max_dev = std::max(box.width, box.height) * 0.3f;
                            std::vector<cv::Point2f> ordered_pts = c_result.points;

                            // === 4 重对称歧义消除 ===
                            if (all_keypoints[idx].size() >= 6 && c_result.points.size() == 4) {
                                float best_err = std::numeric_limits<float>::max();
                                int best_shift = 0;
                                for (int shift = 0; shift < 4; ++shift) {
                                    float err = 0.0f;
                                    for (int i = 0; i < 4; ++i) {
                                        int j = (i + shift) % 4;
                                        float tx = c_result.points[j].x / template_circle_roi_scale_ + c_expanded_roi.x;
                                        float ty = c_result.points[j].y / template_circle_roi_scale_ + c_expanded_roi.y;
                                        float dx = all_keypoints[idx][i + 2].x - tx;
                                        float dy = all_keypoints[idx][i + 2].y - ty;
                                        err += dx * dx + dy * dy;
                                    }
                                    if (err < best_err) {
                                        best_err = err;
                                        best_shift = shift;
                                    }
                                }
                                for (int i = 0; i < 4; ++i) {
                                    ordered_pts[i] = c_result.points[(i + best_shift) % 4];
                                }
                            }
                            // =========================

                            for (size_t i = 0; i < ordered_pts.size() && i < 4; ++i) {
                                float new_x = ordered_pts[i].x / template_circle_roi_scale_ + c_expanded_roi.x;
                                float new_y = ordered_pts[i].y / template_circle_roi_scale_ + c_expanded_roi.y;
                                // 偏差保护：若修正量超过检测框对角线的50%则丢弃
                                float orig_x = all_keypoints[idx][i + 2].x;
                                float orig_y = all_keypoints[idx][i + 2].y;
                                float dx = new_x - orig_x, dy = new_y - orig_y;
                                if (dx * dx + dy * dy < max_dev * max_dev) {
                                    all_keypoints[idx][i + 2].x = new_x;
                                    all_keypoints[idx][i + 2].y = new_y;
                                }
                            }
                        }
                    }
                    } // end enable_template_matching_

                    // ── kp[0] 补帧逻辑 ───────────────────────────────────────
                    if (enable_kp0_hold_) {
                        auto& kp0 = all_keypoints[idx][0];
                        bool kp0_detected = nn_kp0_detected;  // 使用神经网络原始检测结果，不受模板匹配影响
                    
                        // 检查 kp[2-5] 是否全部被检测到
                        bool all_circle_detected = (all_keypoints[idx].size() >= 6);
                        for (int k = 2; k <= 5 && all_circle_detected; ++k) {
                            if (all_keypoints[idx][k].visibility <= 0.5f) all_circle_detected = false;
                        }
                    
                        if (kp0_detected) {
                            // 更新上一帧历史
                            last_kp0_x_ = kp0.x;
                            last_kp0_y_ = kp0.y;
                            has_last_kp0_ = true;
                            if (all_circle_detected) {
                                for (int k = 2; k <= 5; ++k) {
                                    last_circle_kp_x_[k - 2] = all_keypoints[idx][k].x;
                                    last_circle_kp_y_[k - 2] = all_keypoints[idx][k].y;
                                }
                                has_last_circle_kp_ = true;
                            }
                        } else if (has_last_kp0_ && has_last_circle_kp_ && all_circle_detected) {
                            // kp[0] 缺失：计算圆形关键点与上一帧的平均偏移量
                            float avg_dist = 0.0f;
                            for (int k = 2; k <= 5; ++k) {
                                float dx = all_keypoints[idx][k].x - last_circle_kp_x_[k - 2];
                                float dy = all_keypoints[idx][k].y - last_circle_kp_y_[k - 2];
                                avg_dist += std::sqrt(dx * dx + dy * dy);
                            }
                            avg_dist /= 4.0f;
                            if (avg_dist < kp0_hold_max_pixel_dist_) {
                                // 目标位移小，补帧：使用上一帧 kp[0] 坐标
                                kp0.x = last_kp0_x_;
                                kp0.y = last_kp0_y_;
                                kp0.visibility = 1.0f;
                                RCLCPP_DEBUG(this->get_logger(),
                                    "kp[0] hold: last frame used, avg_circle_dist=%.1fpx", avg_dist);
                            }
                        }
                    }
                    // ─────────────────────────────────────────────────────────────

                    // ── kp[0] 红色通道精检测修正 ─────────────────────────
                    if (enable_r_refine_ && all_keypoints[idx].size() > 4 &&
                        all_keypoints[idx][2].visibility > 0.5f &&
                        all_keypoints[idx][4].visibility > 0.5f) {
                        cv::Point2f kp2(all_keypoints[idx][2].x, all_keypoints[idx][2].y);
                        cv::Point2f kp4(all_keypoints[idx][4].x, all_keypoints[idx][4].y);
                        cv::Point2f refined = detect_r_center(image, kp4, kp2);

                        // 偏差保护：若修正量超过检测框对角线的 r_refine_max_dev_ratio_ 则丢弃
                        float max_dev = std::max(box.width, box.height) * r_refine_max_dev_ratio_;
                        float orig_x = all_keypoints[idx][0].x;
                        float orig_y = all_keypoints[idx][0].y;
                        float dx = refined.x - orig_x;
                        float dy = refined.y - orig_y;
                        if (dx * dx + dy * dy < max_dev * max_dev) {
                            all_keypoints[idx][0].x = refined.x;
                            all_keypoints[idx][0].y = refined.y;
                            all_keypoints[idx][0].visibility = 1.0f;
                            RCLCPP_DEBUG(this->get_logger(),
                                "kp[0] refined: NN=(%.1f,%.1f) -> red_ch=(%.1f,%.1f) offset=%.1fpx",
                                orig_x, orig_y, refined.x, refined.y, std::sqrt(dx * dx + dy * dy));
                        }
                        // ── R 检测可视化 ─────────────────────────────────
                        if (enable_visualization_) {
                            // 1. 提取红色通道并二值化（整张图）
                            cv::Mat red_channel;
                            cv::extractChannel(image, red_channel, 0);
                            cv::Mat binary_full;
                            cv::threshold(red_channel, binary_full, r_detect_threshold_, 255, cv::THRESH_BINARY);

                            // 2. 粗估计 R 圆心 + 半径
                            cv::Point2f r_center_raw = (kp2 - kp4) * r_detect_extend_ratio_ + kp2;
                            double r_radius = cv::norm(kp2 - kp4) * r_detect_radius_ratio_;

                            // 3. 创建圆形 Mask 并遮罩
                            cv::Mat roi_mask_full = cv::Mat::zeros(image.size(), CV_8U);
                            cv::circle(roi_mask_full, r_center_raw, static_cast<int>(r_radius), cv::Scalar(255), -1);
                            cv::Mat masked_full;
                            cv::bitwise_and(binary_full, roi_mask_full, masked_full);

                            // 4. 膨胀
                            cv::Mat dilated_full;
                            cv::Mat kernel_r = cv::getStructuringElement(cv::MORPH_RECT,
                                cv::Size(r_detect_dilate_kernel_, r_detect_dilate_kernel_));
                            cv::dilate(masked_full, dilated_full, kernel_r, cv::Point(-1, -1), 1);

                            // 5. 转为 BGR 以便绘制彩色标注
                            cv::Mat r_viz;
                            cv::cvtColor(dilated_full, r_viz, cv::COLOR_GRAY2BGR);

                            // 6. 绘制遮罩圆（绿色圆）
                            cv::circle(r_viz, r_center_raw, static_cast<int>(r_radius), cv::Scalar(0, 255, 0), 2);

                            // 7. 绘制关键点
                            // kp4 (blade_center) - 蓝色
                            cv::circle(r_viz, kp4, 5, cv::Scalar(255, 0, 0), -1);
                            cv::putText(r_viz, "kp4", kp4 + cv::Point2f(8, -8),
                                       cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0), 1);
                            // kp2 (blade_edge) - 青色
                            cv::circle(r_viz, kp2, 5, cv::Scalar(255, 255, 0), -1);
                            cv::putText(r_viz, "kp2", kp2 + cv::Point2f(8, -8),
                                       cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);
                            // 粗估计 R 圆心 - 黄色
                            cv::circle(r_viz, r_center_raw, 5, cv::Scalar(0, 255, 255), -1);
                            cv::putText(r_viz, "r_est", r_center_raw + cv::Point2f(8, -8),
                                       cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1);
                            // 精检测 R 圆心 - 红色
                            cv::circle(r_viz, refined, 5, cv::Scalar(0, 0, 255), -1);
                            cv::putText(r_viz, "R", refined + cv::Point2f(8, -8),
                                       cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);

                            cv::imshow("R Detection", r_viz);
                        }
                        // ─────────────────────────────────────────────────
                    }
                    // ─────────────────────────────────────────────────────

                    geometry_msgs::msg::Pose separator;
                    separator.position.z = -1.0;
                    keypoints_msg.poses.push_back(separator);


                    // 发布关键点：发布 Circle ROI 的特殊点，再发布其他关键点
                    if (has_c_roi) {
                        geometry_msgs::msg::Pose c_tl, c_br;
                        c_tl.position.x = c_expanded_roi.x;
                        c_tl.position.y = c_expanded_roi.y;
                        c_tl.position.z = -2.0;
                        c_br.position.x = c_expanded_roi.x + c_expanded_roi.width;
                        c_br.position.y = c_expanded_roi.y + c_expanded_roi.height;
                        c_br.position.z = -3.0;
                        keypoints_msg.poses.push_back(c_tl);
                        keypoints_msg.poses.push_back(c_br);
                    }
                    // 其他关键点：从 keypoints 0-5 发布，保持与 visualizer.cpp 中的顺序一致（0 是中心点，1-4 是四个角点，5 是额外点）


                    for (size_t k = 0; k < all_keypoints[idx].size() && k < 6; k++) {
                        if (k == 1) continue;

                        const auto& kp = all_keypoints[idx][k];
                        if (kp.visibility > 0.5) {
                            geometry_msgs::msg::Pose pose;
                            pose.position.x = kp.x;
                            pose.position.y = kp.y;
                            pose.position.z = static_cast<double>(k == 0 ? 0 : k - 1);  // 固定映射：kp[0]→0, kp[2]→1, kp[3]→2, kp[4]→3, kp[5]→4
                            keypoints_msg.poses.push_back(pose);
                        }
                    }
                }
                auto t_sift = std::chrono::high_resolution_clock::now();

                keypoints_pub_->publish(keypoints_msg);
                auto t_pub = std::chrono::high_resolution_clock::now();

                RCLCPP_DEBUG(this->get_logger(), "Published %zu keypoints",
                            keypoints_msg.poses.size());

                // 计算各阶段耗时 (毫秒)
                auto ms_convert = std::chrono::duration_cast<std::chrono::microseconds>(t_convert - t_start).count() / 1000.0;
                auto ms_infer   = std::chrono::duration_cast<std::chrono::microseconds>(t_infer   - t_convert).count() / 1000.0;
                auto ms_nms     = std::chrono::duration_cast<std::chrono::microseconds>(t_nms     - t_infer).count() / 1000.0;
                auto ms_sift    = std::chrono::duration_cast<std::chrono::microseconds>(t_sift    - t_nms).count() / 1000.0;
                auto ms_pub     = std::chrono::duration_cast<std::chrono::microseconds>(t_pub     - t_sift).count() / 1000.0;
                auto ms_total   = std::chrono::duration_cast<std::chrono::microseconds>(t_pub     - t_start).count() / 1000.0;

                // 更新 avg_sift_ms EMA（包含模板匹配+kp0补帧+消息组装）
                perf_stats_.avg_sift_ms = 0.9 * perf_stats_.avg_sift_ms + 0.1 * ms_sift;

                static bool logged_first_timing_success = false;
                if (!logged_first_timing_success) {
                    RCLCPP_INFO(this->get_logger(),
#ifdef USE_YOLO26
                        "[Timing first success] total=%.2fms | convert=%.2fms | infer=%.2fms | topk=%.2fms | process(sift+msg)=%.2fms | pub=%.2fms | dets=%zu",
#else
                        "[Timing first success] total=%.2fms | convert=%.2fms | infer=%.2fms | nms=%.2fms | process(sift+msg)=%.2fms | pub=%.2fms | dets=%zu",
#endif
                        ms_total, ms_convert, ms_infer, ms_nms, ms_sift, ms_pub, indices.size());
                    logged_first_timing_success = true;
                } else {
                    RCLCPP_DEBUG(this->get_logger(),
#ifdef USE_YOLO26
                        "[Timing] total=%.2fms | convert=%.2fms | infer=%.2fms | topk=%.2fms | process(sift+msg)=%.2fms | pub=%.2fms | dets=%zu",
#else
                        "[Timing] total=%.2fms | convert=%.2fms | infer=%.2fms | nms=%.2fms | process(sift+msg)=%.2fms | pub=%.2fms | dets=%zu",
#endif
                        ms_total, ms_convert, ms_infer, ms_nms, ms_sift, ms_pub, indices.size());
                }
            } else {
                auto t_end = std::chrono::high_resolution_clock::now();
                auto ms_convert = std::chrono::duration_cast<std::chrono::microseconds>(t_convert - t_start).count() / 1000.0;
                auto ms_infer   = std::chrono::duration_cast<std::chrono::microseconds>(t_infer   - t_convert).count() / 1000.0;
                auto ms_nms     = std::chrono::duration_cast<std::chrono::microseconds>(t_nms     - t_infer).count() / 1000.0;
                auto ms_total   = std::chrono::duration_cast<std::chrono::microseconds>(t_end     - t_start).count() / 1000.0;

                RCLCPP_DEBUG(this->get_logger(),
#ifdef USE_YOLO26
                    "[Timing] total=%.2fms | convert=%.2fms | infer=%.2fms | topk=%.2fms | sift=0.00ms | pub=0.00ms | dets=0",
#else
                    "[Timing] total=%.2fms | convert=%.2fms | infer=%.2fms | nms=%.2fms | sift=0.00ms | pub=0.00ms | dets=0",
#endif
                    ms_total, ms_convert, ms_infer, ms_nms);
            }

            if (enable_visualization_ && indices.size() > 0) {
                cv::Mat vis_image;
                cv::cvtColor(image, vis_image, cv::COLOR_RGB2BGR);  // imshow 期望 BGR，相机发布 RGB

                std::vector<cv::Scalar> kp_colors = {
                    cv::Scalar(0, 0, 255),    // Red - keypoint 0
                    cv::Scalar(255, 0, 0),    // Blue - keypoint 1
                    cv::Scalar(0, 255, 255),  // Yellow - keypoint 2
                    cv::Scalar(255, 0, 255),  // Magenta - keypoint 3
                    cv::Scalar(0, 255, 255),  // Cyan - keypoint 4
                    cv::Scalar(128, 0, 128)   // Purple - keypoint 5
                };

                for (int idx : indices) {
                    if (idx >= all_keypoints.size()) continue;

                    const auto& keypoints = all_keypoints[idx];
                    
                    // 从关键点1-4计算边界框（与visualizer.cpp一致）
                    // 注意：keypoints[1]是中心点，从keypoints[2]开始是四个角点
                    std::vector<cv::Point2f> corner_points;
                    for (size_t k = 2; k < keypoints.size() && k < 6; k++) {
                        if (keypoints[k].visibility > 0.5) {
                            corner_points.push_back(cv::Point2f(keypoints[k].x, keypoints[k].y));
                        }
                    }
                    
                    // 如果有足够的角点，绘制绿色边界框
                    if (corner_points.size() >= 4) {
                        float min_x = std::numeric_limits<float>::max();
                        float min_y = std::numeric_limits<float>::max();
                        float max_x = std::numeric_limits<float>::lowest();
                        float max_y = std::numeric_limits<float>::lowest();
                        for (const auto& pt : corner_points) {
                            min_x = std::min(min_x, pt.x);
                            min_y = std::min(min_y, pt.y);
                            max_x = std::max(max_x, pt.x);
                            max_y = std::max(max_y, pt.y);
                        }
                        cv::rectangle(vis_image,
                                      cv::Point((int)min_x, (int)min_y),
                                      cv::Point((int)max_x, (int)max_y),
                                      cv::Scalar(0, 255, 0), 2);
                    }

                    // 绘制关键点（跳过索引1，与发布逻辑一致）
                    int display_index = 0;
                    for (size_t k = 0; k < keypoints.size() && k < 6; k++) {
                        if (k == 1) continue;

                        const auto& kp = keypoints[k];
                        cv::Point center((int)kp.x, (int)kp.y);
                        cv::Scalar color = kp_colors[display_index % kp_colors.size()];

                        if (kp.visibility > 0.5) {
                            cv::circle(vis_image, center, 5, color, -1);
                            cv::putText(vis_image, std::to_string(display_index),
                                       cv::Point(center.x + 8, center.y - 8),
                                       cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
                            display_index++;
                        }
                    }
                }

                cv::imshow("Detection Visualization", vis_image);
                cv::waitKey(1);
            }

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Processing exception: %s", e.what());
        }
    }

    void run_inference(const cv::Mat& input_image,
                      std::vector<cv::Rect>& boxes,
                      std::vector<float>& confidences,
                      std::vector<int>& class_ids,
                      std::vector<std::vector<Keypoint>>& all_keypoints)
    {
        auto t_preprocess_start = std::chrono::high_resolution_clock::now();
        
        try {
            int orig_width = input_image.cols;
            int orig_height = input_image.rows;

            int target_width = input_width_;
            int target_height = input_height_;

            float scale = std::min((float)target_width / orig_width, (float)target_height / orig_height);
            int new_width = (int)(orig_width * scale);
            int new_height = (int)(orig_height * scale);

            int paste_x = (target_width - new_width) / 2;
            int paste_y = (target_height - new_height) / 2;

            // 使用预分配的图像缓冲区进行 Letterbox resize
            letterboxed_image_.setTo(cv::Scalar(114, 114, 114));

            if (resized_image_.rows != new_height || resized_image_.cols != new_width) {
                resized_image_.create(new_height, new_width, CV_8UC3);
            }

            cv::resize(input_image, resized_image_, cv::Size(new_width, new_height), 0, 0, cv::INTER_LINEAR);
            resized_image_.copyTo(letterboxed_image_(cv::Rect(paste_x, paste_y, new_width, new_height)));

            // 零拷贝构造 u8 NHWC Tensor，颜色转换/归一化/NCHW 转置由 OpenVINO 在设备端完成
            ov::Shape input_shape = {1, static_cast<size_t>(target_height), static_cast<size_t>(target_width), 3};
            ov::Tensor input_tensor(ov::element::u8, input_shape, letterboxed_image_.data);

            auto t_preprocess_end = std::chrono::high_resolution_clock::now();

            // 异步推理
            infer_request.set_input_tensor(input_tensor);
            infer_request.start_async();
            
            // 等待推理完成
            infer_request.wait();

            auto t_inference_end = std::chrono::high_resolution_clock::now();

            ov::Tensor output_tensor = infer_request.get_output_tensor();
            const float* floatarr = output_tensor.data<const float>();
            auto out_shape = output_tensor.get_shape();

            // 优化4: 复用缓冲区，避免重新分配
            boxes_buffer_.clear();
            confidences_buffer_.clear();
            class_ids_buffer_.clear();
            keypoints_buffer_.clear();

#ifdef USE_YOLO26
            // ===== YOLO26 NMS-Free: output shape [1, 300, 24] =====
            // Layout (anchor-major): cols = [x1,y1,x2,y2, conf, cls_id, kp0..kp5]
            // Bbox format: xyxy (top-left, bottom-right), already in 640x640 space
            int num_dets = static_cast<int>(out_shape[1]);   // 300
            int num_cols = static_cast<int>(out_shape[2]);   // 24

            for (int i = 0; i < num_dets; i++) {
                float confidence = floatarr[i * num_cols + 4];
                if (confidence < confidence_threshold_) {
                    continue;
                }

                // xyxy -> top-left x,y and width,height
                float x1 = floatarr[i * num_cols + 0];
                float y1 = floatarr[i * num_cols + 1];
                float x2 = floatarr[i * num_cols + 2];
                float y2 = floatarr[i * num_cols + 3];
                float w = x2 - x1;
                float h = y2 - y1;

                // Reverse letterbox: same as YOLO12
                float x_unpadded = x1 - paste_x;
                float y_unpadded = y1 - paste_y;
                float x_original = x_unpadded / scale;
                float y_original = y_unpadded / scale;
                float w_original = w / scale;
                float h_original = h / scale;

                x_original = std::max(0.0f, std::min(x_original, (float)orig_width));
                y_original = std::max(0.0f, std::min(y_original, (float)orig_height));

                class_ids_buffer_.push_back(0);
                confidences_buffer_.push_back(confidence);
                boxes_buffer_.push_back(cv::Rect((int)x_original, (int)y_original, (int)w_original, (int)h_original));

                // Keypoints: 6 kp × (x, y, vis), starting at channel 6
                std::vector<Keypoint> kps;
                for (int k = 0; k < 6; k++) {
                    float k_x = floatarr[i * num_cols + 6 + k * 3 + 0];
                    float k_y = floatarr[i * num_cols + 6 + k * 3 + 1];
                    float k_vis = floatarr[i * num_cols + 6 + k * 3 + 2];

                    float k_x_orig = (k_x - paste_x) / scale;
                    float k_y_orig = (k_y - paste_y) / scale;

                    k_x_orig = std::max(0.0f, std::min(k_x_orig, (float)orig_width));
                    k_y_orig = std::max(0.0f, std::min(k_y_orig, (float)orig_height));

                    kps.push_back({k_x_orig, k_y_orig, k_vis});
                }
                keypoints_buffer_.push_back(std::move(kps));
            }
#else
            // ===== YOLO12: output shape [1, 23, 8400] =====
            // Layout (channel-major): rows = [cx,cy,w,h, conf, kp0..kp5]
            // Bbox format: cx,cy,w,h (center, width, height), already in 640x640 space
            int num_channels = static_cast<int>(out_shape[1]);  // 23
            int num_anchors  = static_cast<int>(out_shape[2]);  // 8400

            for (int i = 0; i < num_anchors; i++) {
                float confidence = floatarr[4 * num_anchors + i];
                if (confidence < confidence_threshold_) {
                    continue;
                }

                float cx = floatarr[0 * num_anchors + i];
                float cy = floatarr[1 * num_anchors + i];
                float w  = floatarr[2 * num_anchors + i];
                float h  = floatarr[3 * num_anchors + i];

                float x = cx - w / 2.0f;
                float y = cy - h / 2.0f;

                float x_unpadded = x - paste_x;
                float y_unpadded = y - paste_y;
                float x_original = x_unpadded / scale;
                float y_original = y_unpadded / scale;
                float w_original = w / scale;
                float h_original = h / scale;

                x_original = std::max(0.0f, std::min(x_original, (float)orig_width));
                y_original = std::max(0.0f, std::min(y_original, (float)orig_height));

                class_ids_buffer_.push_back(0);
                confidences_buffer_.push_back(confidence);
                boxes_buffer_.push_back(cv::Rect((int)x_original, (int)y_original, (int)w_original, (int)h_original));

                std::vector<Keypoint> kps;
                for (int k = 0; k < 6; k++) {
                    int channel_idx = 5 + k * 3;
                    float k_x = floatarr[(channel_idx + 0) * num_anchors + i];
                    float k_y = floatarr[(channel_idx + 1) * num_anchors + i];
                    float k_vis = floatarr[(channel_idx + 2) * num_anchors + i];

                    float k_x_orig = (k_x - paste_x) / scale;
                    float k_y_orig = (k_y - paste_y) / scale;

                    k_x_orig = std::max(0.0f, std::min(k_x_orig, (float)orig_width));
                    k_y_orig = std::max(0.0f, std::min(k_y_orig, (float)orig_height));

                    kps.push_back({k_x_orig, k_y_orig, k_vis});
                }
                keypoints_buffer_.push_back(std::move(kps));
            }
#endif

            // 将结果移动到输出参数
            boxes = std::move(boxes_buffer_);
            confidences = std::move(confidences_buffer_);
            class_ids = std::move(class_ids_buffer_);
            all_keypoints = std::move(keypoints_buffer_);

            auto t_postprocess_end = std::chrono::high_resolution_clock::now();

            // 更新性能统计
            double preprocess_ms = std::chrono::duration_cast<std::chrono::microseconds>(t_preprocess_end - t_preprocess_start).count() / 1000.0;
            double inference_ms = std::chrono::duration_cast<std::chrono::microseconds>(t_inference_end - t_preprocess_end).count() / 1000.0;
            double postprocess_ms = std::chrono::duration_cast<std::chrono::microseconds>(t_postprocess_end - t_inference_end).count() / 1000.0;
            
            perf_stats_.total_frames++;
            perf_stats_.avg_preprocess_ms = 0.9 * perf_stats_.avg_preprocess_ms + 0.1 * preprocess_ms;
            perf_stats_.avg_inference_ms = 0.9 * perf_stats_.avg_inference_ms + 0.1 * inference_ms;
            perf_stats_.avg_postprocess_ms = 0.9 * perf_stats_.avg_postprocess_ms + 0.1 * postprocess_ms;
            
            // 每100帧输出一次性能统计
            if (perf_stats_.total_frames % 100 == 0) {
                double total_avg = perf_stats_.avg_preprocess_ms + perf_stats_.avg_inference_ms + 
                                 perf_stats_.avg_postprocess_ms + perf_stats_.avg_sift_ms;
                RCLCPP_DEBUG(this->get_logger(),
                    "Perf[frame=%ld] Preprocess=%.2fms Inference=%.2fms Postprocess=%.2fms SIFT=%.2fms Total=%.2fms FPS=%.1f",
                    perf_stats_.total_frames,
                    perf_stats_.avg_preprocess_ms,
                    perf_stats_.avg_inference_ms,
                    perf_stats_.avg_postprocess_ms,
                    perf_stats_.avg_sift_ms,
                    total_avg,
                    1000.0 / total_avg);
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Inference error: %s", e.what());
        }
    }

    // 基于红色通道检测 R 圆心（参照 R_Detector.cpp）
    // blade_center = kp[4]（扇叶中心）, blade_edge = kp[2]（扇叶外沿点）
    cv::Point2f detect_r_center(const cv::Mat& rgb_image,
                                 const cv::Point2f& blade_center,
                                 const cv::Point2f& blade_edge)
    {
        // 1. 粗估计 R 圆心（方向: kp2→kp4, 从 blade_edge 出发沿 blade_edge→blade_center 方向延伸）
        cv::Point2f r_center_t = (blade_edge - blade_center) * r_detect_extend_ratio_ + blade_edge;

        // 2. 计算 Mask 半径
        double radius = cv::norm(blade_center - blade_edge) * r_detect_radius_ratio_;

        // 3. 提取红色通道（RGB 图像，红色是通道 0）
        cv::Mat red_img;
        cv::extractChannel(rgb_image, red_img, 0);

        // 4. 二值化
        cv::Mat binary_img;
        cv::threshold(red_img, binary_img, r_detect_threshold_, 255, cv::THRESH_BINARY);

        // 5. 创建圆形 Mask 并遮罩
        cv::Mat roi_mask = cv::Mat::zeros(rgb_image.size(), CV_8U);
        cv::circle(roi_mask, r_center_t, static_cast<int>(radius), cv::Scalar(255), -1);
        cv::Mat masked_binary;
        cv::bitwise_and(binary_img, roi_mask, masked_binary);

        // 6. 膨胀
        cv::Mat dilated_img;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
            cv::Size(r_detect_dilate_kernel_, r_detect_dilate_kernel_));
        cv::dilate(masked_binary, dilated_img, kernel, cv::Point(-1, -1), 1);

        // 7. 轮廓提取 + minAreaRect 筛选
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(dilated_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        cv::Point2f r_center = r_center_t;
        double best_score = 1e6;
        for (auto& it : contours) {
            auto rotated_rect = cv::minAreaRect(it);
            double ratio = rotated_rect.size.height > rotated_rect.size.width
                             ? rotated_rect.size.height / rotated_rect.size.width
                             : rotated_rect.size.width / rotated_rect.size.height;
            ratio += cv::norm(rotated_rect.center - r_center_t) / (radius / 3.0);
            if (ratio < best_score) {
                best_score = ratio;
                r_center = rotated_rect.center;
            }
        }

        return r_center;
    }

    void extract_channel(const cv::Mat& src, cv::Mat& dst) const
    {
        if (template_channel_mode_ == "red") {
            std::vector<cv::Mat> ch;
            cv::split(src, ch);
            dst = ch[0];  // RGB[0] = 红色通道
        } else if (template_channel_mode_ == "blue") {
            std::vector<cv::Mat> ch;
            cv::split(src, ch);
            dst = ch[2];  // RGB[2] = 蓝色通道
        } else {
            // 默认 gray
            cv::cvtColor(src, dst, cv::COLOR_RGB2GRAY);
        }
    }

    bool match_template_local(const cv::Mat& roi, const TemplateData& td,
                              std::vector<cv::Point2f>& out_pts,
                              cv::Ptr<cv::SIFT> sift_roi) {
        if (roi.empty() || td.descriptors.empty() || !td.matcher) return false;

        std::vector<cv::KeyPoint> kp_roi;
        cv::Mat des_roi;
        sift_roi->detectAndCompute(roi, cv::noArray(), kp_roi, des_roi);
        if (des_roi.empty()) return false;

        // 使用预训练索引：ROI描述子作为query，模板索引已预建，无需每帧重建
        std::vector<std::vector<cv::DMatch>> knn_matches;
        td.matcher->knnMatch(des_roi, knn_matches, 2);

        std::vector<cv::DMatch> good;
        for (auto& m : knn_matches) {
            if (m.size() >= 2 && m[0].distance < td.ratio_test * m[1].distance) good.push_back(m[0]);
        }

        if (good.size() >= (size_t)td.min_matches) {
            std::vector<cv::Point2f> p1, p2;
            for (auto& m : good) {
                // query=des_roi → queryIdx; train=template → trainIdx
                p1.push_back(td.keypoints[m.trainIdx].pt);
                p2.push_back(kp_roi[m.queryIdx].pt);
            }
            cv::Mat H = cv::findHomography(p1, p2, cv::RANSAC, homo_cfg_.ransac_threshold);
            if (!H.empty()) {
                std::vector<cv::Point2f> candidate_pts;
                cv::perspectiveTransform(td.specific_pts, candidate_pts, H);

                // 验证：输出点必须在 ROI 范围内（允许margin_ratio边距），防止单应性矩阵失真导致点飞出
                const float margin = std::max(roi.cols, roi.rows) * homo_cfg_.margin_ratio;
                bool valid = true;
                for (const auto& pt : candidate_pts) {
                    if (pt.x < -margin || pt.x > roi.cols + margin ||
                        pt.y < -margin || pt.y > roi.rows + margin) {
                        valid = false;
                        break;
                    }
                }
                if (valid) {
                    out_pts = candidate_pts;
                    return true;
                }
            }
        }
        return false;
    }

    bool load_sift_params(const std::string& path)
    {
        auto safe_int = [](const YAML::Node& n, int def) -> int {
            return n ? n.as<int>(def) : def;
        };
        auto safe_double = [](const YAML::Node& n, double def) -> double {
            return n ? n.as<double>(def) : def;
        };
        auto safe_float = [](const YAML::Node& n, float def) -> float {
            return n ? n.as<float>(def) : def;
        };

        try {
            YAML::Node config = YAML::LoadFile(path);

            // Circle template
            auto c_node = config["circle_template"];
            if (c_node["specific_pts"]) {
                for (const auto& pt : c_node["specific_pts"]) {
                    if (pt.IsSequence() && pt.size() >= 2) {
                        circle_cfg_.specific_pts.emplace_back(
                            safe_float(pt[0], 0.0f), safe_float(pt[1], 0.0f));
                    }
                }
            }
            if (c_node["sift"]) {
                auto s = c_node["sift"];
                circle_cfg_.sift.nfeatures = safe_int(s["nfeatures"], 0);
                circle_cfg_.sift.nOctaveLayers = safe_int(s["nOctaveLayers"], 3);
                circle_cfg_.sift.contrastThreshold = safe_double(s["contrastThreshold"], 0.01);
                circle_cfg_.sift.edgeThreshold = safe_double(s["edgeThreshold"], 30.0);
                circle_cfg_.sift.sigma = safe_double(s["sigma"], 1.2);
            }
            circle_cfg_.min_matches = safe_int(c_node["min_matches"], 10);
            circle_cfg_.ratio_test = safe_float(c_node["ratio_test"], 0.7f);

            // ROI detector
            auto roi_node = config["roi_detector"]["sift"];
            if (roi_node) {
                roi_sift_cfg_.nfeatures = safe_int(roi_node["nfeatures"], 0);
                roi_sift_cfg_.nOctaveLayers = safe_int(roi_node["nOctaveLayers"], 3);
                roi_sift_cfg_.contrastThreshold = safe_double(roi_node["contrastThreshold"], 0.01);
                roi_sift_cfg_.edgeThreshold = safe_double(roi_node["edgeThreshold"], 30.0);
                roi_sift_cfg_.sigma = safe_double(roi_node["sigma"], 1.2);
            }

            // Homography
            auto h_node = config["homography"];
            homo_cfg_.ransac_threshold = safe_float(h_node["ransac_threshold"], 5.0f);
            homo_cfg_.margin_ratio = safe_float(h_node["margin_ratio"], 0.1f);

            RCLCPP_INFO(this->get_logger(), "SIFT params loaded from: %s", path.c_str());
            return true;
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to load SIFT params from %s: %s. Using hardcoded defaults.", path.c_str(), e.what());
            // Fallback defaults to match original behavior
            circle_cfg_.specific_pts = {
                cv::Point2f(40.0f, 103.0f), cv::Point2f(18.0f, 33.0f),
                cv::Point2f(89.0f, 15.0f), cv::Point2f(110.0f, 85.0f)
            };
            circle_cfg_.min_matches = 10;
            circle_cfg_.ratio_test = 0.7f;
            circle_cfg_.sift.edgeThreshold = 30.0;
            circle_cfg_.sift.sigma = 1.2;

            roi_sift_cfg_.edgeThreshold = 30.0;
            roi_sift_cfg_.sigma = 1.2;

            homo_cfg_.ransac_threshold = 5.0f;
            homo_cfg_.margin_ratio = 0.1f;
            return false;
        }
    }

private:
    ov::Core core;
    ov::CompiledModel compiled_model;
    ov::InferRequest infer_request;
    std::string input_name;
    std::string output_name;


    TemplateData circle_td_;

    TemplateConfig circle_cfg_;
    SiftParams roi_sift_cfg_;
    HomographyConfig homo_cfg_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr keypoints_pub_;

    // /robot_mode 门控（0=SMALL_ENERGY, 1=BIG_ENERGY, 2=ARMOR）
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
    std::atomic<uint8_t> robot_mode_{0};  // 默认 SMALL_ENERGY，serial_driver 启动后快速修正

    std::string model_path_;
    std::string input_topic_;
    std::string output_topic_;
    std::string keypoints_topic_;
    double confidence_threshold_;
    double nms_threshold_;
    int input_width_;
    int input_height_;
    std::string device_;
    int num_threads_;
    std::vector<std::string> class_names_;
    bool enable_visualization_;
    bool enable_template_matching_;
    std::string performance_mode_;

    // kp[0] 补帧状态
    bool has_last_kp0_ = false;
    float last_kp0_x_ = 0.0f;
    float last_kp0_y_ = 0.0f;
    float last_circle_kp_x_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float last_circle_kp_y_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    bool has_last_circle_kp_ = false;
    float kp0_hold_max_pixel_dist_ = 30.0f;
    bool enable_kp0_hold_ = true;
    bool enable_mode_gate_ = true;
    float roi_expand_ratio_ = 8.0f / 3.0f;  // R/Circle ROI 扩展倍数
    float r_roi_base_ratio_ = 3.0f / 5.0f;  // R base box 相对圆形包围盒的比例
    int   r_detect_threshold_ = 17;          // R 红色通道二值化阈值
    float r_detect_radius_ratio_ = 0.6f;     // R Mask 半径比例
    float r_detect_extend_ratio_ = 3.1f;     // R 圆心粗估计延伸倍数
    int   r_detect_dilate_kernel_ = 5;       // R 膨胀核大小
    bool  enable_r_refine_ = true;            // 是否开启 kp[0] 红色通道精检测修正
    float r_refine_max_dev_ratio_ = 0.5f;     // kp[0] 精检测偏差保护比例

    float template_circle_scale_;
    float template_circle_roi_scale_;
    std::string template_channel_mode_;

    // 预分配的图像缓冲区
    cv::Mat letterboxed_image_;                  // 预分配的letterbox图像 (uint8, 640x640)
    cv::Mat resized_image_;                      // 预分配的resize图像 (uint8)
    std::vector<cv::Rect> boxes_buffer_;         // 复用的检测框缓冲区
    std::vector<float> confidences_buffer_;      // 复用的置信度缓冲区
    std::vector<int> class_ids_buffer_;          // 复用的类别缓冲区
    std::vector<std::vector<Keypoint>> keypoints_buffer_; // 复用的关键点缓冲区

    // 性能统计
    struct PerfStats {
        long total_frames = 0;
        double avg_preprocess_ms = 0;
        double avg_inference_ms = 0;
        double avg_postprocess_ms = 0;
        double avg_sift_ms = 0;
    } perf_stats_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(OpenVINOInferenceNode)

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OpenVINOInferenceNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

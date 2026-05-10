#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

// 辅助函数：旋转向量 (rvec) -> 四元数 (Quaternion)
// 为了不引入 tf2 库增加编译复杂度，这里手写一个纯数学转换
geometry_msgs::msg::Quaternion rvecToQuaternion(cv::Mat rvec) {
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    if (R.type() != CV_64F) {
        R.convertTo(R, CV_64F);
    }
    
    geometry_msgs::msg::Quaternion q;
    double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);

    if (trace > 0) {
        double S = sqrt(trace + 1.0) * 2;
        q.w = 0.25 * S;
        q.x = (R.at<double>(2,1) - R.at<double>(1,2)) / S;
        q.y = (R.at<double>(0,2) - R.at<double>(2,0)) / S;
        q.z = (R.at<double>(1,0) - R.at<double>(0,1)) / S;
    } else if ((R.at<double>(0,0) > R.at<double>(1,1)) && (R.at<double>(0,0) > R.at<double>(2,2))) {
        double S = sqrt(1.0 + R.at<double>(0,0) - R.at<double>(1,1) - R.at<double>(2,2)) * 2;
        q.w = (R.at<double>(2,1) - R.at<double>(1,2)) / S;
        q.x = 0.25 * S;
        q.y = (R.at<double>(0,1) + R.at<double>(1,0)) / S;
        q.z = (R.at<double>(0,2) + R.at<double>(2,0)) / S;
    } else if (R.at<double>(1,1) > R.at<double>(2,2)) {
        double S = sqrt(1.0 + R.at<double>(1,1) - R.at<double>(0,0) - R.at<double>(2,2)) * 2;
        q.w = (R.at<double>(0,2) - R.at<double>(2,0)) / S;
        q.x = (R.at<double>(0,1) + R.at<double>(1,0)) / S;
        q.y = 0.25 * S;
        q.z = (R.at<double>(1,2) + R.at<double>(2,1)) / S;
    } else {
        double S = sqrt(1.0 + R.at<double>(2,2) - R.at<double>(0,0) - R.at<double>(1,1)) * 2;
        q.w = (R.at<double>(1,0) - R.at<double>(0,1)) / S;
        q.x = (R.at<double>(0,2) + R.at<double>(2,0)) / S;
        q.y = (R.at<double>(1,2) + R.at<double>(2,1)) / S;
        q.z = 0.25 * S;
    }
    double norm = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (norm > 1e-9) { // 避免除以零
        q.w /= norm;
        q.x /= norm;
        q.y /= norm;
        q.z /= norm;
    } else {
        // 异常情况处理（例如发生）：返回默认单位四元数
        q.w = 1.0; q.x = 0.0; q.y = 0.0; q.z = 0.0;
    }
    return q;
}

int main(int argc, char * argv[]) {
    // 1. 初始化 ROS2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pnp_solver_node");
    
    // ============== 2. 从 YAML 文件加载相机内参 ==============
    std::string camera_name = node->declare_parameter("camera_name", "narrow_stereo");
    auto camera_info_manager =
        std::make_shared<camera_info_manager::CameraInfoManager>(node.get(), camera_name);
    std::string camera_info_url = node->declare_parameter(
        "camera_info_url", "package://vision_bringup/config/camera_info.yaml");

    if (!camera_info_manager->validateURL(camera_info_url)) {
        RCLCPP_FATAL(node->get_logger(), "Invalid camera_info_url: %s", camera_info_url.c_str());
        rclcpp::shutdown();
        return 1;
    }
    camera_info_manager->loadCameraInfo(camera_info_url);
    auto camera_info_msg = camera_info_manager->getCameraInfo();

    cv::Mat cameraMatrix =
        cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_info_msg.k.data())).clone();
    cv::Mat distCoeffs =
        cv::Mat(1, static_cast<int>(camera_info_msg.d.size()), CV_64F,
                const_cast<double *>(camera_info_msg.d.data())).clone();

    RCLCPP_INFO(node->get_logger(), "Loaded camera info from: %s", camera_info_url.c_str());
    RCLCPP_INFO(node->get_logger(),
                "Camera matrix: fx=%.3f, fy=%.3f, cx=%.3f, cy=%.3f",
                cameraMatrix.at<double>(0, 0), cameraMatrix.at<double>(1, 1),
                cameraMatrix.at<double>(0, 2), cameraMatrix.at<double>(1, 2));

    // ✅ 关键修正：为了适配 IPPE 算法，Z 必须为 0 (XY平面)
    // 这里的 X 对应原来的 Y (宽), Y 对应原来的 Z (长)
    // 使用5个点进行PnP解算（移除了原来的 Pt 2）
    std::vector<cv::Point3f> objectPoints = {
        {0.0, 0.0, 0.0},     // Pt 1 (原索引 0)
        {0.0, 0.55, 0.0},    // Pt 2 (原索引 2)
        {0.15, 0.7, 0.0},    // Pt 3 (原索引 3, 左)
        {0.0, 0.85, 0.0},    // Pt 4 (原索引 4)
        {-0.15, 0.7, 0.0}    // Pt 5 (原索引 5, 右)
    };

    // ============== 3. 创建发布者 ==============
    auto publisher = node->create_publisher<geometry_msgs::msg::PoseArray>("/ene_pnp_poses", 10);

    // ============== 4. 创建订阅者 (使用 Lambda 表达式) ==============
    // 创建与发布者匹配的QoS配置
    auto qos_profile = rclcpp::SensorDataQoS();
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

    // 注意：这里的 [=] 表示捕获 main 函数里的所有变量 (如 cameraMatrix, objectPoints 等)
    auto subscription = node->create_subscription<geometry_msgs::msg::PoseArray>(
        "/ene_detection_keypoints", qos_profile,
        [=](const geometry_msgs::msg::PoseArray::SharedPtr msg) {

            // 添加调试日志：记录接收到消息
            RCLCPP_INFO(node->get_logger(), "Received keypoints message with %zu points", msg->poses.size());

            // --- A. 数据检查 ---
            if (msg->poses.empty()) {
                RCLCPP_WARN(node->get_logger(), "Received empty keypoints message. Skipping.");
                return;
            }

            // --- B. 分组关键点：根据分界标记 (-1) 分组 ---
            std::vector<std::vector<cv::Point2f>> detection_groups;
            std::vector<cv::Point2f> current_group;
        
            // 使用 map 存储当前对象的所有点。Key 是 z (0-4)，Value 是 2D 坐标
            // map 会自动按 Key 升序排序，确保 PnP 输入顺序永远是 0,1,2,3,4
            std::map<int, cv::Point2f> sorted_points;

            for (const auto& pose : msg->poses) {
                if (pose.position.z == -1.0) {
                    // 遇到分隔符，说明上一个对象结束了，存入结果
                    if (!sorted_points.empty()) {
                        current_group.clear();
                        for (int i = 0; i < 5; ++i) {
                            if (sorted_points.count(i)) {
                                current_group.push_back(sorted_points[i]);
                            }
                        }
                        if (current_group.size() == 5) {
                            detection_groups.push_back(current_group);
                        }
                        sorted_points.clear();
                    }
                } else if (pose.position.z >= 0.0 && pose.position.z <= 4.0) {
                    // ✅ 核心修正：接受从 0.0 开始的索引
                    sorted_points[(int)pose.position.z] = cv::Point2f(pose.position.x, pose.position.y);
                }
            }

            // 处理最后一组对象（如果消息结尾没有新的 -1.0）
            if (!sorted_points.empty()) {
                current_group.clear();
                for (int i = 0; i < 5; ++i) {
                    if (sorted_points.count(i)) current_group.push_back(sorted_points[i]);
                }
                if (current_group.size() == 5) {
                    detection_groups.push_back(current_group);
                }
            }

            //////////////////

            RCLCPP_INFO(node->get_logger(), "Grouped keypoints into %zu detection objects", detection_groups.size());

            if (detection_groups.empty()) {
                RCLCPP_WARN(node->get_logger(), "No valid detection groups found. Skipping message.");
                return;
            }

            // --- C. 为每组关键点执行PnP求解 ---
            std::vector<geometry_msgs::msg::Pose> pnp_results;

            for (size_t group_idx = 0; group_idx < detection_groups.size(); ++group_idx) {
                const auto& imagePoints = detection_groups[group_idx];

                if (imagePoints.size() < 5) {
                    RCLCPP_WARN(node->get_logger(), "Detection group %zu has only %zu keypoints, need at least 5. Skipping.",
                               group_idx, imagePoints.size());
                    continue;
                }

                // 运行 SOLVEPNP_IPPE
                std::vector<cv::Mat> rvecs, tvecs;
                int solutions = 0;
                try {
                    solutions = cv::solvePnPGeneric(
                        objectPoints,
                        imagePoints,
                        cameraMatrix,
                        distCoeffs,
                        rvecs, tvecs,
                        false,
                        cv::SOLVEPNP_IPPE
                    );
                } catch (cv::Exception& e) {
                    RCLCPP_ERROR(node->get_logger(), "OpenCV Error for group %zu: %s", group_idx, e.what());
                    continue;
                }

                if (solutions < 1) {
                    RCLCPP_WARN(node->get_logger(), "No PnP solutions found for detection group %zu", group_idx);
                    continue;
                }

                // 评估最优解 (Reprojection Error)
                int best_idx = 0;
                double min_error = std::numeric_limits<double>::max();

                for (int i = 0; i < solutions; ++i) {
                    std::vector<cv::Point2f> projected;
                    cv::projectPoints(objectPoints, rvecs[i], tvecs[i], cameraMatrix, distCoeffs, projected);

                    double err = 0.0;
                    for (size_t j = 0; j < imagePoints.size(); ++j) {
                        double dx = imagePoints[j].x - projected[j].x;
                        double dy = imagePoints[j].y - projected[j].y;
                        err += std::sqrt(dx*dx + dy*dy);
                    }
                    err /= imagePoints.size();

                    if (err < min_error) {
                        min_error = err;
                        best_idx = i;
                    }
                }

                // 构建结果Pose
                geometry_msgs::msg::Pose result_pose;
                result_pose.position.x = tvecs[best_idx].at<double>(0);
                result_pose.position.y = tvecs[best_idx].at<double>(1);
                result_pose.position.z = tvecs[best_idx].at<double>(2);
                result_pose.orientation = rvecToQuaternion(rvecs[best_idx]);

                pnp_results.push_back(result_pose);

                RCLCPP_INFO(node->get_logger(),
                           "Detection group %zu PnP Solution - Translation: [%.3f, %.3f, %.3f] m, Reprojection Error: %.3f pixels",
                           group_idx,
                           result_pose.position.x,
                           result_pose.position.y,
                           result_pose.position.z,
                           min_error);
            }

            // --- D. 批量发布所有PnP结果 ---
            auto output_msg = geometry_msgs::msg::PoseArray();
            output_msg.header = msg->header; // 继承时间戳和 Frame ID
            output_msg.poses = pnp_results;

            publisher->publish(output_msg);

            RCLCPP_INFO(node->get_logger(), "Published %zu PnP results from %zu detection groups",
                       output_msg.poses.size(), detection_groups.size());
        }
    );

    RCLCPP_INFO(node->get_logger(), "PnP Solver Node Started (Procedural Style). Waiting for data...");

    // ============== 5. 进入循环 ==============
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
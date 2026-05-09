// src/calibrate_robotworld_handeye.cpp
// 功能：手眼标定主程序，读取采集的图片和IMU数据，计算相机到云台的变换关系
// 【修改】从camera_info.yaml读取相机内参，而不是从calibration.yaml

#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include "tools/img_tools.hpp"
#include "tools/math_tools.hpp"

// 命令行参数定义
const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{camera-info i  | src/vision_bringup/config/camera_info.yaml | 相机内参yaml文件路径 }"  // 【新增】相机内参文件
  "{config-path c  | src/handeye_calibration/config/calibration.yaml | yaml配置文件路径 }"
  "{@input-folder  | src/handeye_calibration/assets/img_with_q        | 输入文件夹路径   }";

/**
 * @brief 从camera_info.yaml读取相机内参
 * @param camera_info_path 相机内参文件路径
 * @param camera_matrix 输出：相机内参矩阵
 * @param distort_coeffs 输出：畸变系数
 * @param logger ROS2日志记录器
 * @return 是否成功读取
 * 
 * 【新增函数】解析ROS标准的camera_info.yaml文件
 */
bool read_camera_info(const std::string & camera_info_path, 
                       cv::Mat & camera_matrix, 
                       cv::Mat & distort_coeffs,
                       rclcpp::Logger logger)
{
  try {
    // 加载YAML文件
    RCLCPP_WARN(logger, "I've reached here!");
    YAML::Node yaml = YAML::LoadFile(camera_info_path);
    // 读取图像尺寸
    int image_width = yaml["image_width"].as<int>();
    int image_height = yaml["image_height"].as<int>();
    RCLCPP_INFO(logger, "相机图像尺寸: %d x %d", image_width, image_height);
    
    // 读取相机内参矩阵 (3x3)
    auto camera_matrix_data = yaml["camera_matrix"]["data"].as<std::vector<double>>();
    if (camera_matrix_data.size() != 9) {
      RCLCPP_ERROR(logger, "camera_matrix数据错误，需要9个元素，实际有%zu个", camera_matrix_data.size());
      return false;
    }
    
    // 构建OpenCV内参矩阵
    camera_matrix = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        camera_matrix.at<double>(i, j) = camera_matrix_data[i * 3 + j];
      }
    }
    
    // 读取畸变系数 (1x5)
    auto distort_coeffs_data = yaml["distortion_coefficients"]["data"].as<std::vector<double>>();
    if (distort_coeffs_data.size() != 5) {
      RCLCPP_WARN(logger, "distortion_coefficients数据大小不是5，实际为%zu，将补零", distort_coeffs_data.size());
      distort_coeffs = cv::Mat(1, 5, CV_64F, cv::Scalar(0));
      for (size_t i = 0; i < distort_coeffs_data.size() && i < 5; i++) {
        distort_coeffs.at<double>(0, i) = distort_coeffs_data[i];
      }
    } else {
      distort_coeffs = cv::Mat(1, 5, CV_64F);
      for (int i = 0; i < 5; i++) {
        distort_coeffs.at<double>(0, i) = distort_coeffs_data[i];
      }
    }
    
    // 输出读取的内参信息
    RCLCPP_INFO(logger, "相机内参矩阵:");
    RCLCPP_INFO(logger, "[%.3f, %.3f, %.3f]", 
                camera_matrix.at<double>(0,0), camera_matrix.at<double>(0,1), camera_matrix.at<double>(0,2));
    RCLCPP_INFO(logger, "[%.3f, %.3f, %.3f]", 
                camera_matrix.at<double>(1,0), camera_matrix.at<double>(1,1), camera_matrix.at<double>(1,2));
    RCLCPP_INFO(logger, "[%.3f, %.3f, %.3f]", 
                camera_matrix.at<double>(2,0), camera_matrix.at<double>(2,1), camera_matrix.at<double>(2,2));
    
    RCLCPP_INFO(logger, "畸变系数: [%.6f, %.6f, %.6f, %.6f, %.6f]",
                distort_coeffs.at<double>(0,0), distort_coeffs.at<double>(0,1),
                distort_coeffs.at<double>(0,2), distort_coeffs.at<double>(0,3),
                distort_coeffs.at<double>(0,4));
    
    return true;
    
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(logger, "解析相机内参文件失败: %s", e.what());
    return false;
  }
}

/**
 * @brief 生成标定板上的三维角点坐标（原点在标定板中心）
 */
std::vector<cv::Point3f> centers_3d(const cv::Size & pattern_size, const float center_distance)
{
  std::vector<cv::Point3f> centers_3d;

  for (int i = 0; i < pattern_size.height; i++) {
    for (int j = 0; j < pattern_size.width; j++) {
      // 原点在标定板中心，Y轴水平向右，Z轴垂直向上
      float x = 0;  // 标定板平面在X=0处
      float y = (-j + 0.5 * pattern_size.width) * center_distance;   // Y坐标：从左到右
      float z = (-i + 0.5 * pattern_size.height) * center_distance;  // Z坐标：从上到下
      centers_3d.push_back({x, y, z});
    }
  }

  return centers_3d;
}

/**
 * @brief 从文本文件中读取四元数
 */
Eigen::Quaterniond read_q(const std::string & q_path)
{
  std::ifstream q_file(q_path);
  double w, x, y, z;
  q_file >> w >> x >> y >> z;
  return {w, x, y, z};
}

/**
 * @brief 加载标定所需的所有数据
 * @param input_folder 输入文件夹路径
 * @param config_path 配置文件路径（包含标定板参数和R_gimbal2imubody）
 * @param camera_matrix 相机内参矩阵（从camera_info.yaml读取）
 * @param distort_coeffs 畸变系数（从camera_info.yaml读取）
 * @param R_gimbal2imubody_data 输出：云台到IMU本体的旋转矩阵
 * @param R_world2gimbal_list 输出：世界到云台的旋转矩阵列表
 * @param t_world2gimbal_list 输出：世界到云台的平移向量列表
 * @param rvecs 输出：PnP解算的相机旋转向量列表
 * @param tvecs 输出：PnP解算的相机平移向量列表
 * @param logger ROS2日志记录器
 * 
 * 【修改】添加camera_matrix和distort_coeffs参数，不再从config读取内参
 */
void load(
  const std::string & input_folder, 
  const std::string & config_path,
  const cv::Mat & camera_matrix,
  const cv::Mat & distort_coeffs,
  std::vector<double> & R_gimbal2imubody_data, 
  std::vector<cv::Mat> & R_world2gimbal_list,
  std::vector<cv::Mat> & t_world2gimbal_list, 
  std::vector<cv::Mat> & rvecs,
  std::vector<cv::Mat> & tvecs, 
  rclcpp::Logger logger)
{
  // 读取YAML配置文件（标定板参数和R_gimbal2imubody）
  auto yaml = YAML::LoadFile(config_path);
  auto pattern_cols = yaml["pattern_cols"].as<int>();           // 标定板列数
  auto pattern_rows = yaml["pattern_rows"].as<int>();           // 标定板行数
  auto center_distance_mm = yaml["center_distance_mm"].as<double>();  // 圆心间距（毫米）
  R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();  // 云台到IMU本体的旋转

  cv::Size pattern_size(pattern_cols, pattern_rows);  // 标定板尺寸
  // 将旋转矩阵数据转换为Eigen矩阵（行主序）
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_gimbal2imubody(R_gimbal2imubody_data.data());

  RCLCPP_INFO(logger, "标定板参数: %d列 x %d行, 圆心距 %.2f mm", 
              pattern_cols, pattern_rows, center_distance_mm);
  RCLCPP_INFO(logger, "开始加载标定数据，期望找到连续的图片文件 (1.jpg, 2.jpg, ...)");
  
  // 循环处理每一组数据，直到找不到图片为止
  for (int i = 1; true; i++) {
    // 构建图片路径和对应的四元数文件路径
    auto img_path = fmt::format("{}/{}.jpg", input_folder, i);
    auto q_path = fmt::format("{}/{}.txt", input_folder, i);
    auto img = cv::imread(img_path);           // 读取图片
    if (img.empty()) {
      RCLCPP_INFO(logger, "已处理 %d 张图片", i - 1);
      break;  // 图片为空则结束循环
    }
    
    Eigen::Quaterniond q = read_q(q_path);     // 读取四元数

    // 计算云台的欧拉角（用于可视化验证）
    Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
    Eigen::Matrix3d R_gimbal2world =
      R_gimbal2imubody.transpose() * R_imubody2imuabs * R_gimbal2imubody;
    Eigen::Vector3d ypr = tools::eulers(R_gimbal2world, 2, 1, 0) * 57.3;  // degree

    // 在图片上显示云台的欧拉角，用于验证R_gimbal2imubody是否正确
    auto drawing = img.clone();
    tools::draw_text(drawing, fmt::format("yaw   {:.2f}", ypr[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(drawing, fmt::format("pitch {:.2f}", ypr[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(drawing, fmt::format("roll  {:.2f}", ypr[2]), {40, 120}, {0, 0, 255});

    // 识别标定板上的圆点网格
    std::vector<cv::Point2f> centers_2d;
    auto success = cv::findChessboardCorners(img, pattern_size, centers_2d);  // 默认对称圆点图案

    // 在图片上绘制识别结果
    cv::drawChessboardCorners(drawing, pattern_size, centers_2d, success);
    cv::resize(drawing, drawing, {}, 0.5, 0.5);
    cv::imshow("Press any to continue", drawing);
    cv::waitKey(500);

    // 输出识别结果
    if (success) {
      RCLCPP_INFO(logger, "[success] %s", img_path.c_str());
    } else {
      RCLCPP_WARN(logger, "[failure] %s", img_path.c_str());
    }
    if (!success) continue;

    // 计算PnP所需的数据
    Eigen::Matrix3d R_world2gimbal = R_gimbal2world.transpose();  // 世界到云台的旋转（转置）
    cv::Mat t_world2gimbal = (cv::Mat_<double>(3, 1) << 0, 0, 0);  // 云台中心为原点
    cv::Mat R_world2gimbal_cv;
    cv::eigen2cv(R_world2gimbal, R_world2gimbal_cv);
    cv::Mat rvec, tvec;  // 相机旋转向量和平移向量
    auto centers_3d_ = centers_3d(pattern_size, center_distance_mm);  // 生成三维角点
    
    // 【修改】使用传入的camera_matrix和distort_coeffs
    cv::solvePnP(
      centers_3d_, centers_2d, camera_matrix, distort_coeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE);

    // 记录本次计算的所有数据
    R_world2gimbal_list.emplace_back(R_world2gimbal_cv);
    t_world2gimbal_list.emplace_back(t_world2gimbal);
    rvecs.emplace_back(rvec);
    tvecs.emplace_back(tvec);
  }
  
  RCLCPP_INFO(logger, "数据加载完成，共获得 %zu 组有效标定数据", rvecs.size());
}

/**
 * @brief 以YAML格式输出手眼标定结果
 */
void print_yaml(
  const std::vector<double> & R_gimbal2imubody_data, 
  const cv::Mat & R_camera2gimbal,
  const cv::Mat & t_camera2gimbal, 
  const Eigen::Vector3d & camera_ypr, 
  double distance,
  const Eigen::Vector3d & board_ypr,
  const std::string & output_path = "")  // 【新增】可选的输出文件路径
{
  YAML::Emitter result;
  
  std::vector<double> R_camera2gimbal_data(
    R_camera2gimbal.begin<double>(), R_camera2gimbal.end<double>());
  std::vector<double> t_camera2gimbal_data(
    t_camera2gimbal.begin<double>(), t_camera2gimbal.end<double>());

  // 构建YAML输出
  result << YAML::BeginMap;
  result << YAML::Key << "R_gimbal2imubody";
  result << YAML::Value << YAML::Flow << R_gimbal2imubody_data;
  result << YAML::Newline;
  result << YAML::Newline;
  result << YAML::Comment(fmt::format(
    "相机同理想情况的偏角: yaw {:.6f} pitch {:.6f} roll {:.6f} degree", 
    camera_ypr[0], camera_ypr[1], camera_ypr[2]));
  result << YAML::Newline;
  result << YAML::Comment(fmt::format(
    "相机同理想情况的偏角: yaw {:.6f} pitch {:.6f} roll {:.6f} rad", 
    camera_ypr[0] * M_PI/180, camera_ypr[1] * M_PI/180, camera_ypr[2] * M_PI/180));
  result << YAML::Newline;
  result << YAML::Comment(fmt::format("标定板到世界坐标系原点的水平距离: {:.2f} m", distance));
  result << YAML::Newline;
  result << YAML::Comment(fmt::format(
    "标定板同竖直摆放时的偏角: yaw {:.2f} pitch {:.2f} roll {:.2f} degree", 
    board_ypr[0], board_ypr[1], board_ypr[2]));
  result << YAML::Key << "R_camera2gimbal";
  result << YAML::Value << YAML::Flow << R_camera2gimbal_data;
  result << YAML::Key << "t_camera2gimbal";
  result << YAML::Value << YAML::Flow << t_camera2gimbal_data;
  result << YAML::Newline;
  result << YAML::EndMap;

  // 输出到控制台
  std::string yaml_str = result.c_str();
  fmt::print("\n{}\n", yaml_str);
  
  // 【新增】如果指定了输出路径，也保存到文件
  if (!output_path.empty()) {
    std::ofstream out_file(output_path);
    out_file << yaml_str;
    out_file.close();
    fmt::print("结果已保存至: {}\n", output_path);
  }
}

/**
 * @brief 主函数
 */
int main(int argc, char * argv[])
{
  // 初始化ROS2节点
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("handeye_calibrator");
  auto logger = node->get_logger();
  
  RCLCPP_INFO(logger, "=== 手眼标定程序启动 ===");
  
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    rclcpp::shutdown();
    return 0;
  }
  auto input_folder = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto camera_info_path = cli.get<std::string>("camera-info");  // 【新增】相机内参文件路径
  
  RCLCPP_INFO(logger, "输入文件夹: %s", input_folder.c_str());
  RCLCPP_INFO(logger, "标定配置文件: %s", config_path.c_str());
  RCLCPP_INFO(logger, "相机内参文件: %s", camera_info_path.c_str());

  // 【新增】读取相机内参
  cv::Mat camera_matrix, distort_coeffs;
  if (!read_camera_info(camera_info_path, camera_matrix, distort_coeffs, logger)) {
    RCLCPP_ERROR(logger, "读取相机内参失败，程序退出");
    rclcpp::shutdown();
    return 1;
  }

  // 从输入文件夹中加载标定所需的数据
  std::vector<double> R_gimbal2imubody_data;
  std::vector<cv::Mat> R_world2gimbal_list, t_world2gimbal_list;
  std::vector<cv::Mat> rvecs, tvecs;
  load(
    input_folder, config_path, camera_matrix, distort_coeffs,  // 【修改】传入相机内参
    R_gimbal2imubody_data, R_world2gimbal_list, t_world2gimbal_list,
    rvecs, tvecs, logger);

  // 检查是否有有效数据
  if (rvecs.empty()) {
    RCLCPP_ERROR(logger, "没有找到有效的标定数据！请检查输入文件夹和配置文件。");
    rclcpp::shutdown();
    return 1;
  }

  // 执行手眼标定
  RCLCPP_INFO(logger, "开始手眼标定计算...");
  cv::Mat R_gimbal2camera, t_gimbal2camera;
  cv::Mat R_world2board, t_world2board;
  cv::calibrateRobotWorldHandEye(
    rvecs, tvecs, R_world2gimbal_list, t_world2gimbal_list, 
    R_world2board, t_world2board,
    R_gimbal2camera, t_gimbal2camera);
  t_gimbal2camera /= 1e3;  // mm to m
  t_world2board /= 1e3;    // mm to m
  
  RCLCPP_INFO(logger, "手眼标定计算完成");

  // 计算所需的数据（转换坐标系）
  cv::Mat R_camera2gimbal, t_camera2gimbal;
  cv::Mat R_board2world, t_board2world;
  cv::transpose(R_gimbal2camera, R_camera2gimbal);
  cv::transpose(R_world2board, R_board2world);
  t_camera2gimbal = -R_camera2gimbal * t_gimbal2camera;
  t_board2world = -R_board2world * t_world2board;

  // 计算相机同理想情况的偏角
  Eigen::Matrix3d R_camera2gimbal_eigen;
  cv::cv2eigen(R_camera2gimbal, R_camera2gimbal_eigen);
  Eigen::Matrix3d R_gimbal2ideal{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
  Eigen::Matrix3d R_camera2ideal = R_gimbal2ideal * R_camera2gimbal_eigen;
  Eigen::Vector3d camera_ypr = tools::eulers(R_camera2ideal, 1, 0, 2) * 57.3;  // degree

  // 计算标定板到世界坐标系原点的水平距离
  auto x = t_board2world.at<double>(0);
  auto y = t_board2world.at<double>(1);
  auto distance = std::sqrt(x * x + y * y);

  // 计算标定板同竖直摆放时的偏角
  Eigen::Matrix3d R_board2world_eigen;
  cv::cv2eigen(R_board2world, R_board2world_eigen);
  Eigen::Vector3d board_ypr = tools::eulers(R_board2world_eigen, 2, 1, 0) * 57.3;  // degree

  // 输出yaml格式的结果（同时保存到文件）
  std::string output_yaml_path = fmt::format("src/handeye_calibration/config/calibration_result.yaml");
  print_yaml(
    R_gimbal2imubody_data, R_camera2gimbal, t_camera2gimbal, 
    camera_ypr, distance, board_ypr, output_yaml_path);

  RCLCPP_INFO(logger, "=== 手眼标定程序完成 ===");
  
  rclcpp::shutdown();
  return 0;
}
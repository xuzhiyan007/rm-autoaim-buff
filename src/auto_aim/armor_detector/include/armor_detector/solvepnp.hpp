#ifndef __OPENRM_SOLVER_SOLVEPNP_H__
#define __OPENRM_SOLVER_SOLVEPNP_H__
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "Eigen/Dense"

namespace auto_aim {

constexpr double ANGLE_UP_15 = M_PI / 12;
constexpr double ANGLE_DOWN_15 = -M_PI / 12;
constexpr double ANGLE_UP_75 = 5 * M_PI / 12;

constexpr double ANGLE_BOUNDARY_UP = M_PI / 4;
constexpr double ANGLE_BOUNDARY_DOWN = 0;

enum ArmorElevation {
    ARMOR_ELEVATION_UP_15,
    ARMOR_ELEVATION_UP_75,
    ARMOR_ELEVATION_DOWN_15,
    ARMOR_ELEVATION_NONE
};

class Camera {

    public:
        Camera() {}
        Camera(
            const std::array<double, 9> & camera_matrix,
            const std::vector<double> & dist_coeffs):
        intrinsic_matrix(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
        distortion_coeffs(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone()){}
        // int width;                                              // 相机的读取分辨率宽
        // int height;                                             // 相机的读取分辨率高
        // int camera_id = 1;                                      // 相机id
        // int file_descriptor;                                    // 相机的文件描述符
    
        // bool flip = false;                                      // 是否翻转图像
    
        // SwapBuffer<Frame>* buffer = nullptr;                    // 帧双缓冲区
    
        // uint32_t capture_buffer_num = 0;                        // 图像读取的缓冲区数量
        // uint32_t* capture_buffer_size = nullptr;                // 图像读取的缓冲区大小, 用于释放内存
        // uint8_t** capture_buffer = nullptr;                     // 图像读取的缓冲区指针
    
        // uint8_t* image_buffer = nullptr;                        // 图像读取的共享内存指针
    
        // uint8_t* rgb_host_buffer = nullptr;                     // 用于推理的主机锁页内存
        // uint8_t* rgb_device_buffer = nullptr;                   // 用于推理的设备显存
    
        std::vector<double> camera_offset;                      // 相机坐标系到云台坐标系的变换
    
        cv::Mat intrinsic_matrix;                               // 相机内参矩阵
        cv::Mat distortion_coeffs;                              // 相机畸变系数
    
        Eigen::Matrix<double, 4, 4> Trans_pnp2world;             // 相机到云台世界的变换矩阵
        Eigen::Matrix<double, 3, 3> Rotate_pnp2world;            // 相机到云台世界的旋转矩阵
        ~Camera() {
            // delete[] capture_buffer;
            // delete[] capture_buffer_size;
            // delete buffer;
        }
    
};

class YawPnP {
public:
    YawPnP() {}
    YawPnP(ArmorElevation elevation) : elevation(elevation) {}

    void setWorldPoints(const std::vector<cv::Point3f>& object_points);
    void setImagePoints(const std::vector<cv::Point2f>& image_points);

    double operator()(double append_yaw) const;

    ArmorElevation setElevation(double pitch);
    // ArmorElevation setElevation(rm::ArmorID armor_id);
    std::vector<Eigen::Vector4d> getMapping(double append_yaw) const;
    std::vector<Eigen::Vector2d> getProject(const std::vector<Eigen::Vector4d>& P_world) const;
    double getCost(const std::vector<Eigen::Vector2d>& P_project, double append_yaw) const;
    double getPixelCost(const std::vector<Eigen::Vector2d>& P_project, double append_yaw) const;
    double getAngleCost(const std::vector<Eigen::Vector2d>& P_project, double append_yaw) const;

    double getCost(double append_yaw) const;
    double getPixelCost(double append_yaw) const;
    double getAngleCost(double append_yaw) const;

    double getYawByPixelCost(double left, double right, double epsilon) const;
    double getYawByAngleCost(double left, double right, double epsilon) const;
    double getYawByMix(double pixel_yaw, double angle_yaw) const;


    double          sys_yaw;
    Eigen::Vector4d pose;
    ArmorElevation  elevation;

    std::vector<Eigen::Vector2d> P_pixel;      // 四点真实像素坐标
    std::vector<Eigen::Vector4d> P_world;      // 四点正对世界坐标

    Eigen::Matrix3d Kc;                        // 相机内参矩阵
    Eigen::Matrix4d T;                         // 图像坐标系在陀螺仪坐标系下的表示
    Eigen::Matrix4d T_inv;                     // 陀螺仪坐标系在图像坐标系下的表示
};


// double solveYawPnP(
//     const double yaw,
//     Camera* camera,
//     Eigen::Vector4d& ret_pose,
//     const std::vector<cv::Point3f>& object_points,
//     const std::vector<cv::Point2f>& image_points,
//     const Eigen::Matrix3d& rotate_head2world,
//     const Eigen::Matrix4d& trans_head2world,
//     // rm::ArmorID armor_id = rm::ARMOR_ID_UNKNOWN,
//     bool display_flag = false);

double solveYawPnP(
    const double yaw,
    std::shared_ptr<Camera> camera,
    Eigen::Matrix3d rotate_pnp,
    Eigen::Vector4d pose_pnp,
    Eigen::Vector4d& ret_pose,
    const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point2f>& image_points,
    bool display_flag = false);
    

void displayYawPnP(YawPnP* yaw_pnp);
void tf_Mat3f(const cv::Mat& input_mat, Eigen::Matrix<double, 3, 3>& output_eigen);
void tf_Mat3d(const cv::Mat& input_mat, Eigen::Matrix<double, 3, 3>& output_eigen);
void tf_Vec4d(const cv::Mat& input_mat, Eigen::Matrix<double, 4, 1>& output_eigen);
void tf_Vec4f(const cv::Mat& input_mat, Eigen::Matrix<double, 4, 1>& output_eigen);
double tf_rotation2armoryaw(const Eigen::Matrix<double, 3, 3>& rotate);
double tf_rotation2armorpitch(const Eigen::Matrix<double, 3, 3>& rotate);
}

#endif
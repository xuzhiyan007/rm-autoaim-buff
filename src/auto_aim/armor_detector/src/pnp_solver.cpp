// Copyright 2022 Chen Jun

#include "armor_detector/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <vector>

namespace auto_aim
{
PnPSolver::PnPSolver(
  const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone())
{
  // Unit: m
  constexpr double small_half_y = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double small_half_z = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;
  constexpr double large_half_y = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double large_half_z = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0;

  // Start from bottom left in clockwise order
  // Model coordinate: x forward, y left, z up
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, -small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, -small_half_z));
  // small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, 0));
  // small_armor_points_.emplace_back(cv::Point3f(0, 0, small_half_z));
  // small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, 0));
  // small_armor_points_.emplace_back(cv::Point3f(0, 0, -small_half_z));

  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, -large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, -large_half_z));
}

bool PnPSolver::solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec)
{
  image_armor_points.clear();
  // y轴图像偏移量,手动在图像坐标系中进行补偿
  // float offset_y = 68;

  // cv::Point2f left_bottom, left_top, right_bottom, right_top;
  // left_bottom = armor.left_light.bottom;
  // left_top = armor.left_light.top;
  // right_top = armor.right_light.top;
  // right_bottom = armor.right_light.bottom;

  // /******************进行补偿*****************/
  // left_bottom.y = left_bottom.y + offset_y;
  // left_top.y = left_top.y + offset_y;
  // right_top.y = right_top.y + offset_y;
  // right_bottom.y = right_bottom.y + offset_y;
  // /******************进行补偿*****************/

  //   // Fill in image point
  // image_armor_points.emplace_back(left_bottom);
  // image_armor_points.emplace_back(left_top);
  // image_armor_points.emplace_back(right_top);
  // image_armor_points.emplace_back(right_bottom);

  // Fill in image points
  // auto armor_center = (armor.left_light.bottom + armor.left_light.top + armor.right_light.top + armor.right_light.bottom) / 4;
  image_armor_points.emplace_back(armor.left_light.bottom);
  image_armor_points.emplace_back(armor.left_light.top);
  image_armor_points.emplace_back(armor.right_light.top);
  image_armor_points.emplace_back(armor.right_light.bottom);  
  
  // image_armor_points.emplace_back((armor.left_light.bottom+armor.left_light.top)/2);
  // image_armor_points.emplace_back((armor.left_light.top+armor.right_light.top)/2);
  // image_armor_points.emplace_back((armor.right_light.top+armor.right_light.bottom)/2);
  // image_armor_points.emplace_back((armor.right_light.bottom+armor.left_light.bottom)/2);  

  // Solve pnp
  object_points_ = armor.type == ArmorType::SMALL ? small_armor_points_ : large_armor_points_;
  return cv::solvePnP(
    object_points_, image_armor_points, camera_matrix_, dist_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point)
{
  float cx = camera_matrix_.at<double>(0, 2);
  float cy = camera_matrix_.at<double>(1, 2);
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

}  // namespace auto_aim

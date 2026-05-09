// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__PNP_SOLVER_HPP_
#define ARMOR_DETECTOR__PNP_SOLVER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core.hpp>

// STD
#include <array>
#include <vector>

#include "armor_detector/armor.hpp"

namespace auto_aim
{
class PnPSolver
{
public:
  PnPSolver(
    const std::array<double, 9> & camera_matrix,
    const std::vector<double> & distortion_coefficients);

  // Get 3d position
  bool solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec);

  // Calculate the distance between armor center and image center
  float calculateDistanceToCenter(const cv::Point2f & image_point);
  
  std::vector<cv::Point3f> object_points_;
  std::vector<cv::Point2f> image_armor_points;

private:
  cv::Mat camera_matrix_;   // IntrinsicMatrix		    fx,fy,cx,cy
  cv::Mat dist_coeffs_;     // DistortionCoefficients k1,k2,p1,p2

  // Unit: mm
  static constexpr float SMALL_ARMOR_WIDTH = 135;
  static constexpr float SMALL_ARMOR_HEIGHT = 55;
  static constexpr float LARGE_ARMOR_WIDTH = 225;
  static constexpr float LARGE_ARMOR_HEIGHT = 55;

  // Four vertices of armor in 3d
  std::vector<cv::Point3f> small_armor_points_;
  std::vector<cv::Point3f> large_armor_points_;

  // The coordinates of the armor in the camera coordinate system 相机<->世界
  cv::Mat ArmorPos;
  // The muzzle coordinates in the camera coordinate system       枪口<->相机
  cv::Mat MuzzlePos;
  // The armor coordinates in the muzzle coordinate system        枪口<->世界
  cv::Mat MuzzlePosUse;
};

}  // namespace auto_aim

#endif  // ARMOR_DETECTOR__PNP_SOLVER_HPP_

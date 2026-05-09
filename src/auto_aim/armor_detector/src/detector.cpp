// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <algorithm>
#include <cmath>
#include <vector>

#include "armor_detector/detector.hpp"
#include "jlcv_interfaces/msg/debug_armor.hpp"
#include "jlcv_interfaces/msg/debug_light.hpp"

namespace auto_aim
{
Detector::Detector(
  const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a)
: binary_thres(bin_thres), enemy_color(color), l(l), a(a)
{
}

std::vector<Armor> Detector::detect(const cv::Mat & input)
{
  binary_img = preprocessImage(input);
  lights_ = findLights(input, binary_img);
  armors_ = matchLights(lights_);

  if (!armors_.empty()) {
    classifier->extractNumbers(input, armors_);
    classifier->classify(armors_);
  }

  return armors_;
}

cv::Mat Detector::getAllNumbersImage()
{
  if (armors_.empty()) {
    return cv::Mat(cv::Size(20, 28), CV_8UC1);
  } else {
    std::vector<cv::Mat> number_imgs;
    number_imgs.reserve(armors_.size());
    for (auto & armor : armors_) {
      number_imgs.emplace_back(armor.number_img);
    }
    cv::Mat all_num_img;
    cv::vconcat(number_imgs, all_num_img);
    return all_num_img;
  }
}

void Detector::drawResults(cv::Mat & img)
{
  // Draw Lights
  for (const auto & light : lights_) {
    cv::circle(img, light.top, 3, cv::Scalar(255, 255, 255), 1);
    cv::circle(img, light.bottom, 3, cv::Scalar(255, 255, 255), 1);
    auto line_color = light.color == RED ? cv::Scalar(255, 255, 0) : cv::Scalar(255, 0, 255);
    cv::line(img, light.top, light.bottom, line_color, 1);
  }

  // Draw armors
  for (const auto & armor : armors_) {
    cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 2);
    cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 2);
  }

  // Show numbers and confidence
  for (const auto & armor : armors_) {
    cv::putText(
      img, armor.classfication_result, armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8,
      cv::Scalar(0, 255, 255), 2);
  }
}

void setLighterHSV(cv::Mat src){
  // cv::imshow("img1", src);
  cv::cvtColor(src, src, cv::COLOR_BGR2HSV);

  std::vector<cv::Mat> channels;
  cv::split(src, channels);

  float brightness_factor = 1.5; 
  channels[2] *= brightness_factor;

  cv::merge(channels, src);

  cvtColor(src, src, cv::COLOR_HSV2BGR);
  
  // cv::imshow("img_", src);

}

cv::Mat Detector::preprocessImage(const cv::Mat & rgb_img)
{
  // cv::Mat gray_img;
  // cv::cvtColor(rgb_img, gray_img, cv::COLOR_RGB2GRAY);

  // cv::Mat binary_img;
  // cv::threshold(gray_img, binary_img, binary_thres, 255, cv::THRESH_BINARY);

  // return binary_img;
  setLighterHSV(rgb_img);
  cv::Mat dst;
  cv::Mat bgr[3];
  cv::Mat img_b, img_g, img_r;
  cv::split(rgb_img, bgr);   // 将三个通道的像素值分离
  img_b = bgr[0];
  img_g = bgr[1];
  img_r = bgr[2];
  // cv::imshow("b", img_b);
  // cv::imshow("g", img_g);
  // cv::imshow("r", img_r);
  cv::Mat imgTargetColor, imgBrightness;
  //  default taget is red
  if (enemy_color == RED) {
    imgTargetColor = img_r - 0.2 * img_g - 0.8 * img_b;
    imgBrightness = img_r;
  } else {
    imgTargetColor = img_b - 0.8 * img_r - 0.2 * img_g;
    imgBrightness = img_b;
  }
  // cv::GaussianBlur(imgBrightness, imgBrightness, cv::Size(3, 3), 0);

  // cv::imshow("bright_thre", imgBrightness);

  // // 拉普拉斯锐化
  // cv::Mat laplacian;
  // cv::Laplacian(imgBrightness, laplacian, CV_16S, 3);
  // cv::convertScaleAbs(laplacian, laplacian);

  // // // // 增强锐化效果：原图 + 拉普拉斯结果
  // // Mat sharpened;
  // cv::addWeighted(imgBrightness, 1.2, laplacian, 0.5, 0, imgBrightness);
  // cv::imshow("bright_rui", imgBrightness);

  // cv::imshow("laplacian", laplacian);


    // cv::Mat gx, gy;
    // cv::Sobel(imgBrightness, gx, CV_32F, 1, 0);  // X方向梯度
    // cv::Sobel(imgBrightness, gy, CV_32F, 0, 1);  // Y方向梯度

    // // // 计算梯度幅值（类似HOG的第一步）
    // cv::Mat magnitude, angle;
    // cv::cartToPolar(gx, gy, magnitude, angle, true);

    // // // 锐化增强：原图 + 梯度幅值
    // cv::normalize(magnitude, magnitude, 0, 255, cv::NORM_MINMAX);
    // magnitude.convertTo(magnitude, CV_8U);
    // addWeighted(imgBrightness, 0.7, magnitude, 0.3, 0, imgBrightness);
    // cv::imshow("magnitude", magnitude);
    // cv::imshow("bright_rui", imgBrightness);

    // 基于梯度方向的边缘筛选（筛选水平/垂直线）
    // Mat edges;
    // Canny(sharpened, edges, 50, 150);

    // // 形态学强化矩形轮廓
    // Mat kernel = getStructuringElement(MORPH_RECT, Size(5,5));
    // morphologyEx(edges, edges, MORPH_CLOSE, kernel);  // 闭合操作连接断裂边缘

    // // 矩形轮廓检测
    // vector<vector<Point>> contours;
    // findContours(edges, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  // // 目标颜色区域
  cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
  cv::dilate(imgTargetColor, imgTargetColor, element);
  // cv::imshow("target_changed", imgTargetColor);
  cv::threshold(imgTargetColor, imgTargetColor, binary_thres, 255, cv::THRESH_BINARY);
  // cv::imshow("target_dialte_thre", imgTargetColor);
  // 亮度区域
  // cv::GaussianBlur(imgBrightness, imgBrightness, cv::Size(3, 3), 1);
  cv::threshold(imgBrightness, imgBrightness, 170, 255, cv::THRESH_BINARY);
  // cv::imshow("bright_thre_rui", imgBrightness);
  // 逻辑与，求交集
  cv::bitwise_and(imgBrightness,imgTargetColor, dst);
  // cv::imshow("dst", dst);
  return dst;
}

std::vector<Light> Detector::findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img)
{
  using std::vector;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  vector<Light> lights;
  this->debug_lights.data.clear();

  for (const auto & contour : contours) {
    if (contour.size() < 5) continue;

    auto r_rect = cv::minAreaRect(contour);
    auto light = Light(r_rect);

    if (isLight(light)) {
      auto rect = light.boundingRect();
      if (  // Avoid assertion failed
        0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rbg_img.cols && 0 <= rect.y &&
        0 <= rect.height && rect.y + rect.height <= rbg_img.rows) {
        int sum_r = 0, sum_b = 0;
        auto roi = rbg_img(rect);
        // Iterate through the ROI
        for (int i = 0; i < roi.rows; i++) {
          for (int j = 0; j < roi.cols; j++) {
            if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
              // if point is inside contour
              sum_r += roi.at<cv::Vec3b>(i, j)[2];
              sum_b += roi.at<cv::Vec3b>(i, j)[0];
            }
          }
        }
        // Sum of red pixels > sum of blue pixels ?
        light.color = sum_r > sum_b ? RED : BLUE;
        lights.emplace_back(light);
      }
    }
  }

  return lights;
}

bool Detector::isLight(const Light & light)
{
  // The ratio of light (short side / long side)
  float ratio = light.width / light.length;
  bool ratio_ok = l.min_ratio < ratio && ratio < l.max_ratio;

  bool angle_ok = light.tilt_angle < l.max_angle;

  bool is_light = ratio_ok && angle_ok;

  // Fill in debug information
  jlcv_interfaces::msg::DebugLight light_data;
  light_data.center_x = light.center.x;
  light_data.ratio = ratio;
  light_data.angle = light.tilt_angle;
  light_data.is_light = is_light;
  this->debug_lights.data.emplace_back(light_data);

  return is_light;
}

std::vector<Armor> Detector::matchLights(const std::vector<Light> & lights)
{
  std::vector<Armor> armors;
  this->debug_armors.data.clear();

  // Loop all the pairing of lights
  for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
    for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
      if (light_1->color != enemy_color || light_2->color != enemy_color) continue;

      if (containLight(*light_1, *light_2, lights)) {
        continue;
      }

      auto type = isArmor(*light_1, *light_2);
      if (type != ArmorType::INVALID) {
        auto armor = Armor(*light_1, *light_2);
        armor.type = type;
        armors.emplace_back(armor);
      }
    }
  }

  return armors;
}

// Check if there is another light in the boundingRect formed by the 2 lights
bool Detector::containLight(
  const Light & light_1, const Light & light_2, const std::vector<Light> & lights)
{
  auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  auto bounding_rect = cv::boundingRect(points);

  for (const auto & test_light : lights) {
    if (test_light.center == light_1.center || test_light.center == light_2.center) continue;

    if (
      bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
      bounding_rect.contains(test_light.center)) {
      return true;
    }
  }

  return false;
}

ArmorType Detector::isArmor(const Light & light_1, const Light & light_2)
{
  // Ratio of the length of 2 lights (short side / long side)
  float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                             : light_2.length / light_1.length;
  bool light_ratio_ok = light_length_ratio > a.min_light_ratio;

  // Distance between the center of 2 lights (unit : light length)
  float avg_light_length = (light_1.length + light_2.length) / 2;
  float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
  bool center_distance_ok = (a.min_small_center_distance <= center_distance &&
                             center_distance < a.max_small_center_distance) ||
                            (a.min_large_center_distance <= center_distance &&
                             center_distance < a.max_large_center_distance);

  // Angle of light center connection
  cv::Point2f diff = light_1.center - light_2.center;
  float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
  bool angle_ok = angle < a.max_angle;

  bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

  // Judge armor type
  ArmorType type;
  if (is_armor) {
    type = center_distance > a.min_large_center_distance ? ArmorType::LARGE : ArmorType::SMALL;
  } else {
    type = ArmorType::INVALID;
  }

  // Fill in debug information
  jlcv_interfaces::msg::DebugArmor armor_data;
  armor_data.type = ARMOR_TYPE_STR[static_cast<int>(type)];
  armor_data.center_x = (light_1.center.x + light_2.center.x) / 2;
  armor_data.light_ratio = light_length_ratio;
  armor_data.center_distance = center_distance;
  armor_data.angle = angle;
  this->debug_armors.data.emplace_back(armor_data);

  return type;
}

}  // namespace auto_aim
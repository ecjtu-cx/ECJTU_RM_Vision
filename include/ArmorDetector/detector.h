#ifndef DETECTOR_H
#define DETECTOR_H

#include <cmath>
#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "armor.h"
#include "serialport.h"
#include "anglesolver.h"
#include "debug.h"

namespace rm_auto_aim {
class Detector {
public:
  /*-------------数据区-----------------------*/
  // 灯条参数
  struct LightParams {
    // 最小面积
    float LightMinArea = 150;
    // 宽高比
    double min_ratio = 0.1;
    double max_ratio = 0.55;
    // 垂直角度
    double max_angle = 40.0;
  };
  // 装甲板参数
  struct ArmorParams {
    double min_light_ratio = 0.6;           // 最小灯条比例
    double min_small_center_distance = 0.8; // 小装甲板中心最小距离
    double max_small_center_distance = 2.8; // 小装甲板中心最大距离
    double min_large_center_distance = 3.2; // 大装甲板中心最小距离
    double max_large_center_distance = 4.6; // 大装甲板中心最大距离
    // 水平角度
    double max_angle = 40.0;      // 装甲板的最大水平角度
    double max_angle_diff = 25.0; // 装甲板的最大角度差异
  };

  /*-------------函数区-----------------------*/
  Detector() = default;
  Detector(const std::string &config_file_path);

  /*-------------数据区-----------------------*/
  int min_lightness; // 最小亮度
  // TODO: 增加数字识别
  // std::unique_ptr<NumberClassifier> classifier;
  cv::Mat debug;                              // 调试图像
  cv::Mat binary_img;                         // 二值图像
  std::vector<Light> True_lights;             // 真实灯条
  std::vector<Armor> True_armors;             // 真实装甲板
  LightParams L_Param;                        // 灯条参数
  ArmorParams A_Param;                        // 装甲板参数
  DetectorState ArmorState = ARMOR_NOT_FOUND; // 装甲板状态
  AngleSolver solver;                        // 角度解算器

  /*-------------函数区-----------------------*/
  // 总识别函数
  void run(cv::Mat &img, int color_label, VisionSendData &data);
  // 根据ROI区域识别
  void ImageByROI(cv::Mat &img);
  // 识别函数
  int detect_for_target(const cv::Mat &frame, int color_label,
                        Armor &TargetArmor);

  void detector(const cv::Mat &input, int enemy_color);
  // 预处理图像
  void PreProcessImage(const cv::Mat &input, cv::Mat &output, int enemy_color);
  // 查找灯条
  void FindLights(const cv::Mat &rbg_img, const cv::Mat &binaryImg,
                  std::vector<Light> &lights);
  // 匹配装甲板
  void matchArmor(const std::vector<Light> &lights, std::vector<Armor> &Armors,
                  int enemy_color);
  // 匹配灯条
  bool containLight(const Light &light_1, const Light &light_2,
                    const std::vector<Light> &lights);
  // 显示图像结果
  void drawResults(cv::Mat &img);
  // 显示调试信息
  void showDebuginfo(float pitch, float yaw, float dis, float XYZ[3]);

private:
  /*---------------函数区-----------------------*/
  bool isLight(const Light &light); // 判断是否为灯条
  bool isArmor(Armor &armor);       // 判断是否为装甲板
};
} // namespace rm_auto_aim

#endif // DETECTOR_H
#ifndef ARMOR_H
#define ARMOR_H

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

namespace rm_auto_aim {
// 装甲板颜色
const int RED = 0;
const int BLUE = 1;
// 灯条装甲板状态
enum DetectorState {
  LIGHTS_NOT_FOUND = 0,
  LIGHTS_FOUND = 1,
  ARMOR_NOT_FOUND = 2,
  ARMOR_FOUND = 3
};
enum ArmorType { SMALL = 0, LARGE = 1 };
// 灯条结构体
struct Light : public cv::RotatedRect {
  Light() = default;

  explicit Light(cv::RotatedRect box) : cv::RotatedRect(box) {
    // 灯条四个顶点
    cv::Point2f p[4];
    box.points(p);
    // 灯条y坐标排序
    std::sort(p, p + 4, [](const cv::Point2f &a, const cv::Point2f &b) {
      return a.y < b.y;
    });

    top = (p[0] + p[1]) / 2;
    bottom = (p[2] + p[3]) / 2;

    length = cv::norm(top - bottom);
    width = cv::norm(p[0] - p[1]);

    tilt_angle =
        std::atan2(std::abs(top.y - bottom.y), std::abs(top.x - bottom.x));
    tilt_angle = tilt_angle / CV_PI * 180;
  }
  /*-------------灯条参数-----------------------*/
  int color;               // 灯条颜色
  cv::Point2f top, bottom; // 灯条上下端点
  double length;           // 灯条长度
  double width;            // 灯条宽度
  float tilt_angle;        // 灯条倾斜角
  /*-------------灯条参数-----------------------*/
};
// 装甲板
struct Armor {
  Armor() = default;

  Armor(const Light &l1, const Light &l2) {
    if (l1.center.x < l2.center.x) {
      left_light = l1;
      right_light = l2;
    } else {
      left_light = l2;
      right_light = l1;
    }
    center = (left_light.center + right_light.center) / 2;
    std::vector<cv::Point2f> vertexes({left_light.top, left_light.bottom,
                                       right_light.bottom, right_light.top});
    vertex = vertexes;
  }

  /*-------------装甲板参数----------------------*/
  Light left_light, right_light; // 左右灯条
  cv::Point2f center;            // 装甲板中心

  cv::Mat number;                   // 装甲板数字
  int idx;                          // 装甲板编号
  float similarity;                 // 装甲板相似度
  float confidence;                 // 装甲板置信度
  std::vector<cv::Point2f> vertex;  // 装甲板四个顶点
  std::string classfication_result; // 装甲板分类结果
  ArmorType armor_type;             // 装甲板类型
  /*-------------装甲板参数----------------------*/
};

} // namespace rm_auto_aim

#endif // ARMOR_H
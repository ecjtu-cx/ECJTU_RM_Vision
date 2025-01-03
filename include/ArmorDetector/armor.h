#ifndef ARMOR_H
#define ARMOR_H
#include <iostream>
#include <opencv2/opencv.hpp>

namespace rm_auto_aim
{
    // 装甲板颜色
    const int RED = 0;
    const int BLUE = 1;
    // 灯条装甲板状态
    enum DetectorState
    {
        LIGHTS_NOT_FOUND = 0,
        LIGHTS_FOUND = 1,
        ARMOR_NOT_FOUND = 2,
        ARMOR_FOUND = 3
    };
    // 装甲板类型
    enum ArmorType
    {
        SMALL = 0,
        LARGE = 1
    };
    // 灯条
    struct Light : public cv::RotatedRect
    {
        Light() = default;

        explicit Light(cv::RotatedRect box)
            : cv::RotatedRect(box)
        {
            // 灯条四个顶点
            cv::Point2f p[4];
            box.points(p);
            // 灯条y坐标排序
            std::sort(p, p + 4, [](const cv::Point2f &a, const cv::Point2f &b)
                      { return a.y < b.y; });
            top = (p[0] + p[1]) / 2;
            bottom = (p[2] + p[3]) / 2;

            length = cv::norm(top - bottom);
            width = cv::norm(p[0] - p[1]);

            tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
            tilt_angle = tilt_angle / CV_PI * 180;
        }

        int color;
        cv::Point2f top, bottom;
        double length;
        double width;
        float tilt_angle;
    };
    // 装甲板
    struct Armor
    {
        Armor() = default;

        Armor(const Light &l1, const Light &l2)
        {
            if (l1.center.x < l2.center.x)
            {
                left_light = l1, right_light = l2;
            }
            else
            {
                left_light = l2, right_light = l1;
            }
            center = (left_light.center + right_light.center) / 2;
            std::vector<cv::Point2f> vertexes(
                {left_light.top, left_light.bottom, right_light.bottom, right_light.top});
            vertex = vertexes;
        }

        Light left_light, right_light;
        cv::Point2f center;

        cv::Mat number_img;
        std::string number;
        int idx;
        float similarity;
        float confidence;
        std::vector<cv::Point2f>
            vertex; // 装甲板的四个顶点，不包括光条区域
        std::string classfication_result;
        ArmorType armor_type;
    };
}

#endif
#ifndef DETETCTOR_H
#define DETETCTOR_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "armor.h"
#include "number_classifier.h"
#include "debug.h"
#include "serialport.h"
#include "anglesolver.h"
namespace rm_auto_aim
{
    class Detector
    {
    public:
        struct LightParams
        {
            float LightMinArea = 150;
            // width / height
            double min_ratio = 0.1;
            double max_ratio = 0.55;
            // vertical angle
            double max_angle = 40.0;
        };
        struct ArmorParams
        {
            double min_light_ratio = 0.6;
            double min_small_center_distance = 0.8;
            double max_small_center_distance = 2.8;
            double min_large_center_distance = 3.2;
            double max_large_center_distance = 4.6;
            // horizontal angle
            double max_angle = 40.0;
            double max_angle_diff = 25.0;
        };
        Detector() = default;
        Detector(const std::string &config_file_path);

        int min_lightness;
        std::unique_ptr<NumberClassifier>
            classifier;
        cv::Mat debug;
        cv::Mat binary_img;
        std::vector<Light> True_lights;
        std::vector<Armor> True_armors;
        LightParams L_Param;
        ArmorParams A_Param;
        DetectorState ArmorState = ARMOR_NOT_FOUND;
        // 姿态解算类
        AngleSolver solver;

        void run(cv::Mat &img, int color_label, VisionSendData &data);
        void ImageByROI(cv::Mat &img);
        int detect_for_target(const cv::Mat &frame, int color_label, Armor &TargetArmor);
        void detector(const cv::Mat &input, int enemy_color);
        void PreProcessImage(const cv::Mat &input, cv::Mat &output, int enemy_color);
        void FindLights(const cv::Mat &rbg_img, const cv::Mat &binaryImg, std::vector<Light> &lights);
        void matchArmor(const std::vector<Light> &lights, std::vector<Armor> &Armors, int enemy_color);
        bool containLight(const Light &light_1, const Light &light_2, const std::vector<Light> &lights);
        void drawResults(cv::Mat &img);
        void showDebuginfo(float pitch, float yaw, float dis, float XYZ[3]);

    private:
        bool isLight(const Light &light);
        bool isArmor(Armor &armor);
    };

}

#endif
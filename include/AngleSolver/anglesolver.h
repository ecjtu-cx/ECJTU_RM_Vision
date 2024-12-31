#ifndef ANGLESOLVER_H
#define ANGLESOLVER_H
#include "armor.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <opencv2/core/eigen.hpp> //放在eigen头文件后面
#include <opencv2/opencv.hpp>
using namespace std;
// armor size
#define BIG_ARMOR_LEN 217.5
#define BIG_ARMOR_WID 49.5
#define SMALL_ARMOR_LEN 137.2
#define SMALL_ARMOR_WID 54.1
class AngleSolver {
private:
  float pitch, yaw, dis, bias_yaw, bias_pitch, bias_dis;
  std::vector<cv::Point3f> BigArmorPoint3D;
  std::vector<cv::Point3f> SmallArmorPoint3D;
  cv::Mat camera_matrix = cv::Mat(3, 3, CV_32FC1);
  cv::Mat distortion_coefficients;
  Eigen::Vector3d xyz;
  float _cam_bias_x;
  float _cam_bias_z;
  float _cam_bias_y;
  float _coeff_friction;
  float _gravity;

public:
  void Init(const std::string &paramPath, float camBiasX, float camBiasY,
            float camBiasZ, float gravity);          // 结算初始化
  void solve_angle(rm_auto_aim::Armor &TargetArmor); // 姿态结算计算
  void GetAngle(float &pitch, float &yaw, float &distance,
                float XYZ[3]); // 获取角度距离信息接口
};

#endif
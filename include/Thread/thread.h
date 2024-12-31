#ifndef THREAD_H
#define THREAD_H

#include "Camera.h"
#include "detector.h"
#include "debug.h"
#include "anglesolver.h"
#include "serialport.h"
#include "factor.h"

#include <iostream>
#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

// 任务数据
struct TaskData
{
    int mode;
    double bullet_speed;
    Mat img;
    Eigen::Quaterniond quat;
    int timestamp; // 单位：ms
};

class ThreadManager
{
private:
    // 相机类
    HDURM::HKcam hkcam;
    // TODO: 增加串口通信
    //SerialPort serial;
    std::unique_ptr<rm_auto_aim::Detector> autoAim;

public:
    ThreadManager() = default;

    void InitManager(const std::string &config_file_path);
    bool producer(Factory<TaskData> &factory);
    bool consumer(Factory<TaskData> &factory);
    // TODO : 增加串口通信
    //bool consumer(Factory<TaskData> &factory, Factory<VisionSendData> &transmit_factory);
    //bool dataTransmitter(Factory<VisionSendData> &transmit_factory);
};

#endif
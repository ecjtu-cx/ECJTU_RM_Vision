#ifndef THREAD_H
#define THREAD_H

#include "Camera.h"
#include "anglesolver.h"
#include "debug.h"
#include "detector.h"
#include "factor.h"
#include "serialport.h"

#include <iostream>
#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

class ThreadManager {
private:
  // 相机类
  HDURM::HKcam hkcam;
  SerialPort serial;
  std::unique_ptr<rm_auto_aim::Detector> autoAim;

public:
  ThreadManager() = default;

  void InitManager(const std::string &config_file_path);
  bool producer(Factory<TaskData> &factory);
  bool consumer(Factory<TaskData> &factory,
                Factory<VisionSendData> &transmit_factory);
  bool dataTransmitter(Factory<VisionSendData> &transmit_factory);
};

#endif
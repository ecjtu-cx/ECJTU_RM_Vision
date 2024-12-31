#include "thread.h"

void ThreadManager::InitManager(const std::string &config_file_path) {
  autoAim = std::make_unique<rm_auto_aim::Detector>(config_file_path);
}

bool ThreadManager::producer(Factory<TaskData> &factory) {

  hkcam.OpenCam("2BDFA2166410");
  hkcam.SetParam();

  while (1) {
    // 产生数据
    TaskData src;
    auto ret = hkcam.GetFlame(src.img);

    if (!ret) {
      break;
    }

    factory.produce(src);
  }

  hkcam.CloseCam();
  return true;
}

bool ThreadManager::consumer(Factory<TaskData> &factory) {
  while (1) {
    TaskData dst;
    // 发送串口数据
    VisionSendData data;
    // 从缓存区取数据
    factory.consume(dst);
    /*-----------------------------------自瞄---------------------------------------------------*/
    double t1 = (double)cv::getTickCount();
#if DETEDRED
    autoAim->run(dst.img, rm_auto_aim::RED, data);
#else
    autoAim->run(dst.img, rm_auto_aim::BLUE, data);
#endif
    // transmit_factory.produce(data);
    double t2 = (double)cv::getTickCount();
    int fps = cv::getTickFrequency() / (t2 - t1);
    /*-----------------------------------自瞄----------------------------------------------------*/
  }
  return true;
}
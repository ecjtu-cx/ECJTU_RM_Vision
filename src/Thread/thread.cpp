#include "thread.h"
// 生产者

void ThreadManager::InitManager(const std::string &config_file_path)
{
  autoAim = std::make_unique<rm_auto_aim::Detector>(config_file_path);
}

bool ThreadManager::producer(Factory<TaskData> &factory)
{

  hkcam.OpenCam("2BDFA2166410");
  hkcam.SetParam();

  while (1)
  {
    // 产生数据
    TaskData src;
    auto ret = hkcam.GetFlame(src.img);

    if (!ret)
    {
      break;
    }

    factory.produce(src);
  }

  hkcam.CloseCam();
  return true;
}
// 消费者
bool ThreadManager::consumer(Factory<TaskData> &factory, Factory<VisionSendData> &transmit_factory)
{
  auto mode = 0, last_mode = 0;
  // 自瞄类

  while (1)
  {
    TaskData dst;
    // 发送串口数据
    VisionSendData data;
    // 从缓存区取数据
    factory.consume(dst);

    // if (mode != last_mode)
    // {
    //     fmt::print(fmt::fg(fmt::color::pale_violet_red), "[CONSUMER] Mode switched to {}\n", mode);
    //     last_mode = mode;
    // }

    /*-----------------------------------自瞄---------------------------------------------------*/
    double t1 = (double)cv::getTickCount();
#ifdef DETEDRED
    autoAim->run(dst.img, rm_auto_aim::RED, data);
#else
    autoAim->run(dst.img, rm_auto_aim::BLUE, data);
#endif
    // fmt::print(fmt::fg(fmt::color::blue), "Pitch: {} Yaw: {} Dis: {} \n", data.pitch_angle.f, data.yaw_angle.f, data.dis.f);
    transmit_factory.produce(data);
    double t2 = (double)cv::getTickCount();
    int fps = cv::getTickFrequency() / (t2 - t1);
    /*-----------------------------------自瞄----------------------------------------------------*/
  }
  return true;
}
// 串口发送线程
bool ThreadManager::dataTransmitter(Factory<VisionSendData> &transmit_factory)
{
  SerialPort serial("/dev/ttyUSB0", 921600);
  while (1)
  {
    VisionSendData data;
    transmit_factory.consume(data);
    // 若串口离线即初始化失败则跳过数据发送
    // TODO:使用无串口的模式时会导致此线程死循环，浪费CPU性能
    if (serial.need_init == true)
    {
      usleep(5000);
      continue;
    }
    //fmt::print(fmt::fg(fmt::color::blue), "i send data\n");
    serial.send(data);
  }
  return true;
}
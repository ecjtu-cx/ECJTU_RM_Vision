#include "thread.h"

int main()
{
  // 工厂模板类
  Factory<TaskData> task_factory(3);
  Factory<VisionSendData> data_transmit_factory(5);

  ThreadManager thread_manager;

  char cwd[PATH_MAX];
  if (getcwd(cwd, sizeof(cwd)) == nullptr) {
    fmt::print(fmt::fg(fmt::color::red), "fail to getcwd(Hkcam-SetParam)\n");
    return 0;
  }
  
  string path = cwd + string("/Config/CameraParam.xml");
  fmt::print(fmt::fg(fmt::color::green), "CameraParam path: {}\n", path);
  thread_manager.InitManager(path.c_str());
  /*--------串口发送线程----------*/
  thread transmitter(&ThreadManager::dataTransmitter, &thread_manager, std::ref(data_transmit_factory));
  fmt::print(fmt::fg(fmt::color::blue), "Transmitter start !!!!!!!!!!\n");
  /*--------相机更新线程---------*/
  thread task_producer(&ThreadManager::producer, &thread_manager, std::ref(task_factory));
  fmt::print(fmt::fg(fmt::color::blue), "Producer start !!!!!!!!!!\n");
  /*--------自瞄线程--------*/
  thread task_consumer(&ThreadManager::consumer, &thread_manager, std::ref(task_factory), std::ref(data_transmit_factory));
  fmt::print(fmt::fg(fmt::color::blue), "Consumer start !!!!!!!!!!\n");

  transmitter.join();
  task_producer.join();
  task_consumer.join();
  return 0;
}


#include "thread.h"
#include <fmt/color.h>
#include <string>
#include <unistd.h>

int main() {
  Factory<TaskData> task_factory(3);

  ThreadManager thread_manager;

  char cwd[PATH_MAX];
  if (getcwd(cwd, sizeof(cwd)) == nullptr) {
    fmt::print(fmt::fg(fmt::color::red), "fail to getcwd(Hkcam-SetParam)\n");
    return 0;
  }
  string path = cwd + string("/Config/CameraParam.xml");
  fmt::print(fmt::fg(fmt::color::green), "CameraParam path: {}\n", path);
  thread_manager.InitManager(path.c_str());
  /*--------相机更新线程---------*/
  thread task_producer(&ThreadManager::producer, &thread_manager,
                       std::ref(task_factory));
  fmt::print(fmt::fg(fmt::color::blue), "Producer start !!!!!!!!!!\n");
  /*--------自瞄线程--------*/
  thread task_consumer(&ThreadManager::consumer, &thread_manager,
                       std::ref(task_factory));
  fmt::print(fmt::fg(fmt::color::blue), "Consumer start !!!!!!!!!!\n");

  task_producer.join();
  task_consumer.join();
  return 0;
}
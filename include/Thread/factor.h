#ifndef FACTOR_H
#define FACTOR_H

#include <deque>
#include <mutex>
#include <unistd.h>

template <typename T> class Factory {
private:
  std::deque<T> buffer;
  int buffer_size;
  std::mutex lock;

public:
  /**
   * @brief 工厂类初始化
   * @param size 队列长度
   **/
  Factory(int size) { buffer_size = size; };
  bool produce(T &product);
  bool consume(T &product);
};

template <typename T> bool Factory<T>::produce(T &product) {

  lock.lock();                     // 加锁
  if (buffer.size() < buffer_size) // 数据缓存区未满，放入缓存区末尾
    buffer.push_back(product);
  else {
    buffer.pop_front(); // 满了删除缓存区头部元素，再放在缓存区末尾
    buffer.push_back(product);
  }
  lock.unlock(); // 解锁

  return true;
}

template <typename T> bool Factory<T>::consume(T &product) {
  while (1) {
    lock.lock();
    if (!buffer.empty())
      break;
    lock.unlock();
    usleep(1e3);
  }
  product = buffer.front();
  buffer.pop_front();
  lock.unlock();

  return true;
}

#endif // FACTOR_H
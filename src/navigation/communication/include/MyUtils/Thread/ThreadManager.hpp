//用于在程序退出时通知所有线程关闭
#pragma once
#include "ThreadSafeContainer.hpp"
#include <future>
#include <memory>
#include <pthread.h>
#include <signal.h>
#include <thread>
using namespace std;
namespace MyUtils {
namespace Thread {
class ThreadManager
{
private:
  ThreadSafeQueue<std::thread::native_handle_type> q;

private:
  explicit ThreadManager() : q() {}
  ThreadManager(ThreadManager &&) = delete;
  ThreadManager(const ThreadManager &) = delete;
  ThreadManager operator=(const ThreadManager &) = delete;

public:
  static ThreadManager & getInstance()
  {
    static ThreadManager a;
    return a;
  }
  ~ThreadManager()
  {
    while (q.size()) {
      pthread_cancel(q.pop());
    }
  }
  /**
   * @brief 要求必须要在线程detach 或者 join 之前获取native_handle
   * @param [in] t 线程号，可以由thread.native_handle()获得
   */
  void add(const std::thread::native_handle_type & t) { q.put(t); }

  template <class F, class... Args> auto add(F && f, Args &&... args) -> std::future<decltype(f(args...))>
  {
    using RetType = decltype(f(args...));  // typename std::result_of<F(Args...)>::type, 函数 f
                                           // 的返回值类型
    auto task = std::make_shared<std::packaged_task<RetType()>>(bind(forward<F>(f),
      forward<Args>(args)...));  // 把函数入口及参数,打包(绑定)

    future<RetType> future = task->get_future();
    std::thread t([task]() {
      (*task)();
    });
    q.put(t.native_handle());
    t.detach();
    return future;
  }

private:
};
}  // namespace Thread
}  // namespace MyUtils

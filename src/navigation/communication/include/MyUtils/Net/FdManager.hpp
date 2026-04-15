/**
 * @brief 参考资料 https://www.cnblogs.com/WindSun/p/12142656.html
 */

#pragma once
#include <event.h>
#include <event2/thread.h>
#include <event2/util.h>
#include <map>
#include <set>
#include <signal.h>

#include "../DataType/BiMap.hpp"
#include "../DataType/ByteArray.hpp"
#include "../Thread/ThreadManager.hpp"
#include "../Timer/Timer.hpp"

#include <iostream>
// #include "protocol.hpp"

#ifdef _DEBUG
#include <iostream>
#endif

namespace MyUtils {
namespace Net {
using MyUtils::DataType::ByteArray;
typedef std::shared_ptr<bufferevent> shared_bev;
// typedef void ( *fd_read_cb )( ByteArray, void * );
typedef std::function<void(ByteArray)> fd_read_cb;

class FdWrapper
{
  friend class FdManager;

private:
  struct TimerData
  {
    // ByteArray m_data;
  public:
    std::weak_ptr<MyUtils::MyTimer::__detail::Timer> m_timer;
    MyUtils::MyTimer::TimerManager * m_timer_manager;
    explicit TimerData(MyUtils::MyTimer::TimerManager * timer_manager,
      std::weak_ptr<MyUtils::MyTimer::__detail::Timer> timer) noexcept
    : m_timer(timer), m_timer_manager(timer_manager)
    {
    }
    ~TimerData()
    {
      // MYLOG_INFO( "timerdata析构" );
      m_timer_manager->delTimer(m_timer);
    }
  };

private:
  MyUtils::MyTimer::TimerManager m_timer_manager;
  std::thread::native_handle_type m_handle_timer_manager;
  std::map<uint16_t, TimerData> m_seqnum2data;
  shared_bev m_bev;
  size_t m_rtt;  //一次报文往返时间,单位毫秒
  size_t m_rto;  //超时重传时间,单位毫秒
  std::atomic_bool m_is_activate;
  std::string m_name;  //连接名

  fd_read_cb m_read_cb;

private:
  FdWrapper(const std::string & name, shared_bev bev, fd_read_cb read_cb)
  : m_timer_manager(), m_seqnum2data(), m_bev(bev), m_rtt(100), m_rto(2 * m_rtt), m_is_activate(true),
    m_name(name), m_read_cb(read_cb)
  {
    // TODO this指针的安全性
    std::thread t([this]() {
      this->m_timer_manager.run();
    });
    m_handle_timer_manager = t.native_handle();
    t.detach();

    // this->m_timer_manager.addTimer( 1000, true, [ name ]( ) {
    //     MYLOG_INFO( "fd:%s timer is running", name.c_str( ) );
    // } );
  }

public:
  ~FdWrapper() { pthread_cancel(m_handle_timer_manager); }

private:
  shared_bev bev() const { return m_bev; }

  inline void ack(const uint16_t seq_num) { m_seqnum2data.erase(seq_num); }

  /**
   * @brief 将发送出去的报文加入到待确认区域中，并设置超时
   *
   */
  inline void insert(const uint16_t seq_num, const ByteArray & data)
  {
    shared_bev bev = this->m_bev;

    std::weak_ptr<MyUtils::MyTimer::__detail::Timer> timer =
      this->m_timer_manager.addTimer(m_rto, false, [bev, data, this]() {
        if (bufferevent_write(bev.get(), data.get(), data.size()) == -1) {
          this->m_is_activate = false;
        }
      });

    m_seqnum2data.insert(std::make_pair(seq_num, TimerData(&m_timer_manager, timer)));
  }

  inline void erase(const uint16_t seq_num) { this->m_seqnum2data.erase(seq_num); }
};

// TODO 线程安全
class FdManager
{
private:
private:
  struct event_base * m_base;
  struct event * m_signal_event;
  MyUtils::DataType::BiMap<int, bufferevent *> m_fd2bev;
  MyUtils::DataType::BiMap<int, FdWrapper *> m_fd2wrapper;  //句柄到 FdWrapper 的映射
  MyUtils::DataType::BiMap<int, std::string> m_fd2name;     //句柄到命名的映射

public:
  /**
   * @brief
   * @param [in] rto 超时重传时间，以毫秒为单位
   */
  explicit FdManager() : m_base(nullptr), m_signal_event(nullptr), m_fd2bev(), m_fd2wrapper(), m_fd2name()
  {
    evthread_use_pthreads();  //会自动添加锁机制
    m_base = event_base_new();
    if (m_base != nullptr) {
      m_signal_event = evsignal_new(m_base, SIGINT, __signal_cb, (void *)this);
    }
  }
  ~FdManager() { __signal_cb(SIGINT, 0, this); }

  /**
   * @brief 启动监听程序。该函数应当放在一个单独的线程中阻塞执行
   */
  inline void run() { event_base_loop(m_base, EVLOOP_NO_EXIT_ON_EMPTY); }

  /**
   * @brief 添加待监听的fd.
   * @param [in] name 连接名字
   * @param [in] fd 文件描述符
   * @param [in] readcb read回调函数，可以参考本文件中的样例
   * @param [in] writecb write回调函数，一般置为NULL
   * @param [in] eventcb 异常处理回调函数，传入参数为NULL时将使用默认处理函数
   * @param [in] cbarg 前三个回调函数的传入参数
   * @throw 当出现重复添加时会抛出异常
   * @return bufferevent *
   */
  void add(const std::string & name,
    int fd,
    fd_read_cb readcb,
    bufferevent_data_cb writecb = nullptr,
    bufferevent_event_cb eventcb = nullptr,
    void * cbarg = nullptr)
  {
    // TODO 修改传入参数
    (void)writecb;  // 避免未使用参数警告
    (void)eventcb;  // 避免未使用参数警告
    (void)cbarg;    // 避免未使用参数警告
    //检查fd是否重复插入
    auto it = this->m_fd2name.find_by_first(fd);
    if (it != this->m_fd2name.end()) {
      char temp[1024];
      sprintf(temp, "%d 已经被添加进来了\n", fd);
      throw(temp);
      return;
    }
    //检查是否连接重名
    it = this->m_fd2name.find_by_second(name);
    if (it != this->m_fd2name.end()) {
      throw("连接命名重复");
      return;
    }

    this->m_fd2name.insert(std::make_pair(fd, name));
    shared_bev bev(bufferevent_socket_new(m_base, fd, BEV_OPT_CLOSE_ON_FREE | BEV_OPT_THREADSAFE),
      [](bufferevent * bev) {
        bufferevent_free(bev);
      });

    bufferevent_setcb(bev.get(), __read_cb, nullptr, __event_cb, (void *)this);

    bufferevent_enable(bev.get(), EV_READ | EV_PERSIST);

    m_fd2bev.insert(std::make_pair(fd, bev.get()));

    FdWrapper * wrapper = new FdWrapper(name, bev, readcb);

    this->m_fd2wrapper.insert(std::make_pair(fd, wrapper));
    return;
  }

  size_t size() const { return this->m_fd2name.size(); }

  /**
   * @brief 查找是否存在名为name的连接
   * @param [in] name 连接名
   * @return true 表示存在，否则不存在
   */
  bool exist(const std::string & name)
  {
    return (!(this->m_fd2name.find_by_second(name) == this->m_fd2name.end()));
  }

  /**
   * @brief 确认序列号
   */
  void ack(const std::string & name, const uint16_t seq_num)
  {
    auto it = this->m_fd2name.find_by_second(name);
    auto it2 = this->m_fd2wrapper.find_by_first(it->first);
    it2->second->ack(seq_num);
  }

  /**
   * @brief 移除对应名称的连接
   * @param [in] name 连接名称
   */
  void remove(const std::string & name)
  {
    auto it = this->m_fd2name.find_by_second(name);
    if (it != this->m_fd2name.end()) {
      __remove(it->first);
    } else {
      std::string r("名为\"" + name + "\"的连接不存在");
      throw(r);
    }
    return;
  }

private:
  void __remove(int fd)
  {
    this->m_fd2name.erase_by_first(fd);
    this->m_fd2bev.erase_by_first(fd);
    auto it2 = this->m_fd2wrapper.find_by_first(fd);
    delete it2->second;
    this->m_fd2wrapper.erase_by_first(fd);
    return;
  }

public:
  /**
   * @brief 向名为name的fd发送消息
   * @param [in] name 连接名
   * @param [in] data 需要发送的数据
   * @param [in] data_len 数据长度
   * @return 0 if successful, -2 the name is error, -1 error occurred when
   * send data
   */
  int send(const std::string & name, const char * data, const size_t & data_len) noexcept
  {
    // TODOD
    // 线程安全。这里安全分为两部分，第一部分是m_name2bev的读安全，第二部分是bufferevent_write的写安全
    // 但是在启用evthread_use_pthreads之后，bufferevent_write本身是线程安全的，故而不再考虑
    // MyUtils::print(data, data_len);

    auto it = this->m_fd2name.find_by_second(name);
    if (it != this->m_fd2name.end()) {
      // 0 if successful, or -1 if an error occurred
      auto it2 = this->m_fd2wrapper.find_by_first(it->first);

      return bufferevent_write(it2->second->bev().get(), data, data_len);
    } else {
      // std::cerr << "Name " << name << " not found in fd_manager!" << std::endl;
      return -2;
    }
  }

private:
  static void __event_cb(struct bufferevent * bev, short what, void * arg)
  {
    if (what & BEV_EVENT_EOF) {
    } else if (what & BEV_EVENT_ERROR) {
    } else if (what & BEV_EVENT_CONNECTED) {
      return;
    }
    //这将自动close套接字和free读写缓冲区
    FdManager * ths = (FdManager *)arg;
    ths->__remove(ths->m_fd2bev.find_by_second(bev)->first);
  }
  static void __signal_cb(evutil_socket_t sig, short events, void * user_data)
  {
    (void)sig;     // 避免未使用参数警告
    (void)events;  // 避免未使用参数警告
    printf("Caught an interrupt signal; exiting cleanly in two "
           "seconds.\n");

    FdManager * ths = (FdManager *)user_data;

    for (auto it = ths->m_fd2name.begin(); it != ths->m_fd2name.end(); ++it) {
      ths->remove(it->second);
    }

    event_free(ths->m_signal_event);
    event_base_loopbreak(ths->m_base);
    event_base_free(ths->m_base);
  }

  static void __read_cb(struct bufferevent * bev, void * arg)
  {
    char msg[4096];
    size_t len = bufferevent_read(bev, msg, sizeof(msg));

    FdManager * ths = (FdManager *)arg;
    auto it = ths->m_fd2wrapper.find_by_first(ths->m_fd2bev.find_by_second(bev)->first);
    it->second->m_read_cb(ByteArray(msg, len));
  }
};

}  // namespace Net

}  // namespace MyUtils

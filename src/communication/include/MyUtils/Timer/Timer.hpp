#pragma once
#include "../Thread/ThreadPoll.hpp"
#include "../Thread/ThreadSafeContainer.hpp"
#include "../tools.hpp"
#include <functional>
#include <mutex>
#include <queue>
#include <sys/epoll.h>

namespace MyUtils
{

  /**
   * 本代码的源码以及代码分析见下面链接，本人在前人工作的基础上做了一些改进
   * https://gist.github.com/baixiangcpp/b2199f1f1c7108f22f47d2ca617f6960
   * https://www.cnblogs.com/sunsky303/p/14154190.html
   */

  namespace MyTimer
  {

    using call_back_fun = std::function<void()>;  //回调函数

    template <typename TDuration, typename TF, class... TArgs>
    static std::result_of_t<TF && (TArgs && ...)> run_with_timeout(TDuration timeout, TF&& f, TArgs&&... args)
    {
      using R = std::result_of_t<TF && (TArgs && ...)>;
      std::packaged_task<R(TArgs...)> task(f);
      auto future = task.get_future();
      std::thread thr(std::move(task), std::forward<TArgs>(args)...);
      if(future.wait_for(timeout) != std::future_status::timeout)
      {
        thr.join();
        return future.get();  // this will propagate exception from f() if any
      }
      else
      {
        thr.detach();  // we leave the thread still running
        throw std::runtime_error("Timeout");
      }
    }

    namespace __detail
    {

      class Timer
      {
      private:
        call_back_fun fun;
        bool is_loop_execution;
        atomic_bool m_is_deleted;  //延时删除
        long long expire_;
        unsigned timeout_;

      public:
        Timer() = delete;
        template <class F>
        Timer(long long timeout, bool _is_loop_execution, F&& f)
          : fun(f)
          , is_loop_execution(_is_loop_execution)
          , m_is_deleted(false)
          , timeout_(timeout)
        {
          expire_ = Tools::getCurrentMillisecs() + timeout;
          // MYLOG_INFO( "timer 是否循环执行 %s",
          //             is_loop_execution ? "true" : "false" );
          // MYLOG_INFO( "timer  m_is_deleted is %s",
          //             m_is_deleted ? "true" : "false" );
        }

      public:
        ~Timer()
        {
          // MYLOG_INFO("调用Timer析构");
          is_loop_execution = false;
          m_is_deleted = true;
        }

      public:
        inline void active()
        {
          fun();
        }

        inline long long getExpire() const
        {
          return expire_;
        }

        inline void resetTime()
        {
          if(!is_loop_execution)
            return;
          expire_ += timeout_;
        }
        inline void deletTimer()
        {
          // MYLOG_INFO("删除定时器");
          m_is_deleted = true;
        }
        inline bool is_deleted() const
        {
          return m_is_deleted.load();
        }
        inline bool is_loop() const
        {
          return is_loop_execution;
        }
      };
    }  // namespace __detail

    /**
     * 用于管理定时器，采用单例模式进行管理，避免多线程的问题
     * 线程安全的定时器管理器.在定时器中调用定时器管理器是不明智的，会造成死锁，因此
     * 在添加定时器时，不允许在其中调用定时器管理的任何功能
     */
    class TimerManager
    {
    private:
      using Timer = __detail::Timer;
      typedef std::shared_ptr<__detail::Timer> SharedTimerPtr;

      struct cmp
      {
        bool operator()(const SharedTimerPtr& lhs, const SharedTimerPtr& rhs) const
        {
          return (lhs)->getExpire() > (rhs)->getExpire();
        }
      };

      typedef std::priority_queue<SharedTimerPtr, std::vector<SharedTimerPtr>, cmp> min_heap;

      typedef Thread::ThreadSafeHeap<SharedTimerPtr, std::vector<SharedTimerPtr>, cmp> thread_safe_min_heap;

    private:
      const int MAXEVENTS = 1024;  //最多支持1024个定时器

      min_heap m_queue_;                       //当前活动的定时器
      thread_safe_min_heap m_newbuffer_queue;  //用于添加定时器时的缓冲区

    public:
      TimerManager()
        : m_newbuffer_queue()
      {
      }
      TimerManager(const TimerManager&) = delete;
      TimerManager(TimerManager&&) = delete;
      TimerManager& operator=(const TimerManager&) = delete;

      ~TimerManager() = default;

      /**
       * @brief 必须要将该函数放在一个全新的线程中执行
       */
      void run()
      {
        int efd = epoll_create1(0);
        struct epoll_event* events = new epoll_event[MAXEVENTS];
        while(true)
        {
          int n = epoll_pwait(efd,
                              events,
                              MAXEVENTS,
                              __getRecentTimeout(),
                              NULL);  //每隔2000毫秒唤醒一次，防止程序无法退出
          (void)n;                    // 避免未使用变量警告
          // if ( n == -1 && errno == EINTR )
          // {
          //     MYLOG_INFO( "timer 退出" );
          //     break;
          // }
          __takeAllTimeout();
        }
      }

      /**
       * @brief
       * 添加定时器。定时器中执行的任务最好不要是阻塞执行的。因为定时器本质上是轮询。
       * @param timeout [in] 超时设置,多久之后会超时。单位毫秒
       * @param is_loop_execution [in]
       * 是否循环执行。如果为FALSE的话，超时之后只会执行一次；如果为TRUE的话，从当前时刻开始，每隔timeout时间就执行一次
       * @param f [in] 回调函数.返回值要求为void
       * @param args [in] 回调函数需要的参数
       * @return weak_ptr<Timer*>
       * 这里使用智能指针的意义在于防止用户释放timer，这样也可以将用户从内存管理中解放
       */
      template <class F, class... Args>
      weak_ptr<__detail::Timer> addTimer(long long timeout, bool is_loop_execution, F&& f, Args&&... args)
      {
        if(timeout == 0)
          return SharedTimerPtr(NULL);

        auto fun = std::bind(std::forward<F>(f), std::forward<Args>(args)...);

        SharedTimerPtr p(new Timer(timeout, is_loop_execution, fun), [](Timer* timer) { delete timer; });

        m_newbuffer_queue.put(p);

        return p;
      }

      void delTimer(weak_ptr<__detail::Timer> timer)
      {
        auto obj = timer.lock();  //线程安全的
        if(obj)
        {
          obj->deletTimer();
        }
        return;
      }

    private:
      /**
       * @brief
       * 当没有定时器添加进列表时，返回值为100，使得定时器管理器能够每隔100ms扫描一次定时器列表
       * 当有定时器加入时，返回值是最近的定时器减去当前时间得到的差值
       */
      long long __getRecentTimeout()
      {
        long long timeout = 100;
        if(m_queue_.size() == 0)
          return timeout;

        long long now = Tools::getCurrentMillisecs();
        timeout = (m_queue_.top())->getExpire() - now;
        if(timeout < 0)
          timeout = 0;

        return timeout;
      }

      /**
       * @brief 执行所有已超时的定时器
       */
      void __takeAllTimeout()
      {
        //首先将缓冲区里面的定时器填充到m_queue中

        while(m_newbuffer_queue.size())
        {
          auto p = m_newbuffer_queue.try_pop();
          m_queue_.push(p);
        }

        min_heap newqueue;

        while(!m_queue_.empty())
        {
          SharedTimerPtr timer = m_queue_.top();
          m_queue_.pop();  //这里必须将取出的timer删除掉
          if(timer->is_deleted())
          {
            // MYLOG_INFO( "timer 被删除了" );
            continue;  //不执行已经删除的timer，不将timer放入newqueue就删除了timer了
          }
          if(timer->getExpire() <= Tools::getCurrentMillisecs())
          {
            // 这里会阻塞执行
            timer->active();
            if((timer)->is_loop())
            {
              (timer)->resetTime();
              if(!timer->is_deleted())  //判断定时器是否被删除了
              {
                newqueue.push(timer);
              }
              else
              {
              }
            }
          }
          else
          {
            newqueue.push(timer);
          }
        }

        m_queue_ = newqueue;
        // MYLOG_INFO( "m_queue_ size is %lld", m_queue_.size( ) );
        return;
      }
    };
  }  // namespace MyTimer

}  // namespace MyUtils

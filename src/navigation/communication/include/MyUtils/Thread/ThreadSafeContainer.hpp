#pragma once
#include <boost/smart_ptr.hpp>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <stack>
namespace MyUtils {
namespace Thread {

namespace __detail {

template <typename _T, typename _Sequence> class ThreadSafeContainer
{
protected:
  std::timed_mutex mt_mutex;  //互斥访问__spaces
  boost::shared_ptr<_Sequence> __spaces;
  std::condition_variable_any m_cond;

public:
  /**
   * @brief 构造共享空间
   * @param N 共享空间大小
   */
  explicit ThreadSafeContainer() : __spaces(new _Sequence()) {}

  /**
   * @brief 因为要进行加锁，所以不能使用const修饰
   */
  ThreadSafeContainer(ThreadSafeContainer & obj) { *this = obj; }

  ThreadSafeContainer(ThreadSafeContainer && obj) { *this = obj; }

  ThreadSafeContainer & operator=(ThreadSafeContainer && other)
  {
    if (this == &other)
      return *this;
    //这里是非常有必要的。我们设想一种情况，有两个全局对象a,b和两个线程
    //如果在一号线程里执行了a = b, 在二号线程里面执行了b = a
    //并且不是始终按照低地址对象(按照高地址对象加锁也可以)顺序加锁
    //那么就会发生死锁。一号线程先加锁了a，二号线程先加锁了b。
    //这样一号线程就会等待二号线程释放b的锁，而二号线程则会等待一号释放a的锁
    // ThreadSafeContainer &l = ( this < &other ) ? ( *this ) : other;
    // ThreadSafeContainer &r = ( this > &other ) ? ( *this ) : other;
    // std::unique_lock< std::timed_mutex > l_lock( l.mt_mutex );
    // std::unique_lock< std::timed_mutex > r_lock( r.mt_mutex );
    // this->__filled_space_sem = other.__filled_space_sem;
    // this->__spaces = other.__spaces;
    // return *this;

    // c++17提供了一种方式可以避免不同线程之间上锁的问题，不再需要自己处理
    std::scoped_lock<std::timed_mutex, std::timed_mutex> lock{this->mt_mutex, other.mt_mutex};
    this->__spaces = other.__spaces;
    return *this;
  }

  ThreadSafeContainer & operator=(ThreadSafeContainer & other)
  {
    *this = std::move(other);
    return *this;
  }

  ThreadSafeContainer & copy(ThreadSafeContainer && other)
  {
    ThreadSafeContainer & l = (this < &other) ? (*this) : other;
    ThreadSafeContainer & r = (this > &other) ? (*this) : other;
    std::unique_lock<std::timed_mutex> l_lock(l.mt_mutex);
    std::unique_lock<std::timed_mutex> r_lock(r.mt_mutex);
    this->__spaces = boost::shared_ptr<_Sequence>(new _Sequence(*(other.__spaces)));
    return *this;
  }

  ThreadSafeContainer & copy(ThreadSafeContainer & other)
  {
    this->copy(std::move(other));
    return *this;
  }

  ~ThreadSafeContainer() {}

  void put(const _T & p) noexcept
  {
    std::unique_lock<std::timed_mutex> lock(mt_mutex);
    this->__push(p);
    m_cond.notify_one();
    return;
  }

  /**
   * @brief
   * 值得注意的是，这里的数据在被取出之后，容器里面就不再包含该数据.阻塞执行
   */
  _T pop() noexcept
  {
    std::unique_lock<std::timed_mutex> lock(mt_mutex);
    while (size() == 0)
      m_cond.wait(lock, [this] {
        return this->size() > 0;
      });
    _T ret = this->__pop();
    return ret;
  }

  /**
   * @brief 非阻塞函数.如果timeout_duration不为0则为阻塞函数
   * @param [in] timeout_duration
   * 超时时间限制，单位毫秒。该函数会在timeout_duration内执行完毕
   * @throw if -1 no data in buffer and -2 when to fail to lock mutex
   * @return
   */
  _T try_pop(const int & timeout_duration = 0)
  {
    // If timeout_duration is less or equal timeout_duration.zero(), the
    // function behaves like try_lock()
    // std::unique_lock<std::timed_mutex>< std::mutex > lock(
    //     mt_mutex, std::chrono::milliseconds( timeout_duration ) );
    if (this->mt_mutex.try_lock_for(std::chrono::milliseconds(timeout_duration))) {
      if (size() == 0) {
        this->mt_mutex.unlock();
        throw(-1);  // no data to get
      } else {
        _T ret = this->__pop();
        this->mt_mutex.unlock();
        return ret;
      }
    } else {
      throw(-2);  // cannot get the lock
    }
  }

  /**
   * @brief
   * 值得注意的是get函数在调用之后除非调用pop函数否则数据仍然存在.阻塞执行
   */
  const _T & get() noexcept
  {
    std::unique_lock<std::timed_mutex> lock(mt_mutex);
    while (size() == 0)
      m_cond.wait(lock, [this] {
        return this->size() > 0;
      });
    const _T & ret = this->__get();
    return ret;
  }

  /**
   * @brief 非阻塞函数.如果timeout_duration不为0则为阻塞函数
   * @param [in] timeout_duration
   * 超时时间限制，单位毫秒。该函数会在timeout_duration内执行完毕
   * @throw if -1 no data in buffer and -2 when to fail to lock mutex
   * @return
   */
  const _T & try_get(const int & timeout_duration = 0)
  {
    // If timeout_duration is less or equal timeout_duration.zero(), the
    // function behaves like try_lock()
    if (this->mt_mutex.try_lock_for(std::chrono::milliseconds(timeout_duration))) {
      if (size() == 0) {
        this->mt_mutex.unlock();
        throw(-1);  // no data to get
      } else {
        const _T & ret = this->__get();
        this->mt_mutex.unlock();
        return ret;
      }
    } else {
      throw(-2);  // cannot get the lock
    }
  }

  size_t size() const { return this->__spaces->size(); }

protected:
  virtual inline void __push(const _T & t) = 0;

  virtual inline _T __pop() = 0;

  virtual inline const _T & __get() = 0;
};
}  // namespace __detail

template <typename _T> class ThreadSafeQueue : public __detail::ThreadSafeContainer<_T, std::queue<_T>>
{
private:
public:
  explicit ThreadSafeQueue() : __detail::ThreadSafeContainer<_T, std::queue<_T>>() {}

  // ThreadSafeQueue( const ThreadSafeQueue & ) = delete;
  // ThreadSafeQueue &operator=( const ThreadSafeQueue & ) = delete;

protected:
  inline void __push(const _T & t) override
  {
    this->__spaces->push(t);
    return;
  }

  inline _T __pop() override
  {
    _T ret = __get();
    this->__spaces->pop();

    return ret;
  }

  inline const _T & __get() override { return this->__spaces->front(); }
};

template <typename _T> class ThreadSafeStack : public __detail::ThreadSafeContainer<_T, std::stack<_T>>
{
private:
public:
  explicit ThreadSafeStack() : __detail::ThreadSafeContainer<_T, std::stack<_T>>() {}

private:
  inline void __push(const _T & t) override
  {
    this->__spaces->push(t);
    return;
  }
  inline _T __pop() override
  {
    _T ret = __get();
    this->__spaces->pop();
    return ret;
  }

  inline const _T & __get() override { return this->__spaces->top(); }
};

template <typename _T,
  typename _Sequence = std::vector<_T>,
  typename _Compare = std::less<typename _Sequence::value_type>>
class ThreadSafeHeap
: public __detail::ThreadSafeContainer<_T, std::priority_queue<_T, _Sequence, _Compare>>
{
private:
  typedef __detail::ThreadSafeContainer<_T, std::priority_queue<_T, _Sequence, _Compare>> parent_class;

public:
  explicit ThreadSafeHeap() : parent_class() {}

private:
  inline void __push(const _T & t) override
  {
    this->__spaces->push(t);
    return;
  }
  inline _T __pop() override
  {
    _T ret = __get();
    this->__spaces->pop();
    return ret;
  }
  inline const _T & __get() override { return this->__spaces->top(); }
};

}  // namespace Thread

}  // namespace MyUtils

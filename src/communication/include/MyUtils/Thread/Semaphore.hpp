#pragma once
#include <condition_variable>
#include <mutex>
#include <chrono>
namespace MyUtils
{
namespace Thread
{
class Semaphore
{
  private:
    atomic_int32_t count;
    std::mutex mt_mutex;
    std::condition_variable cond;

  public:
    explicit Semaphore( int count = 0 ) : count( count ) {}
    ~Semaphore( ) = default;
    void signal( )
    {
        std::unique_lock< std::mutex > lock( mt_mutex );
        ++count;
        cond.notify_one( );
        return;
    }
    void signal_all( )
    {
        std::unique_lock< std::mutex > lock( mt_mutex );
        ++count;
        cond.notify_all( );
        return;
    }
    void wait( )
    {
        std::unique_lock< std::mutex > lock( mt_mutex );
        cond.wait( lock, [ = ] { return count > 0; } );
        --count;
        return;
    }
    // void wait_for( const std::chrono::duration< std::chrono::Rep, Period > &rel_time )
    // {
    //     if ( count > 0 )
    //         return;
    //     std::unique_lock< std::mutex > lock( mt_mutex );
    //     cond.wait_for( lock, rel_time, [ = ] { return count > 0; } );
    //     --count;
    //     return;
    // }
};
} // namespace Thread

} // namespace MyUtils

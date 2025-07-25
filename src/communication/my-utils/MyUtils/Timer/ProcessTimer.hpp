#pragma once

#include "../tools.hpp"

namespace MyUtils
{
namespace MyTimer
{
class ProcessTimer
{
  private:
    int64_t m_start_time;

  public:
    ProcessTimer( ) : m_start_time( getCurrentMillisecs( ) ) {}

    /**
     * @brief 获取到从开始到目前位置的运行时间
     * @return 返回值为运行时间，单位毫秒
     */
    int64_t duration( ) const
    {
        return getCurrentMillisecs( ) - this->m_start_time;
    }

    /**
     * @brief 重置计时器
    */
    void reset( ) { m_start_time = getCurrentMillisecs( ); }

    ~ProcessTimer( ) {}
};

} // namespace MyTimer

} // namespace MyUtils

#pragma once
#include "color_text.hpp"
#include <algorithm>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>

namespace custom_log {
inline std::string now_string()
{
  using namespace std::chrono;
  const auto now = system_clock::now();
  const auto now_t = system_clock::to_time_t(now);
  std::tm tm_buf{};
#if defined(_WIN32)
  localtime_s(&tm_buf, &now_t);
#else
  localtime_r(&now_t, &tm_buf);
#endif
  const auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
  std::ostringstream ts;
  ts << std::setfill('0') << std::setw(2) << tm_buf.tm_hour << ':' << std::setw(2) << tm_buf.tm_min << ':'
     << std::setw(2) << tm_buf.tm_sec << '.' << std::setw(3) << ms.count();
  return ts.str();
}
template <typename T> struct named_value
{
  std::string_view name;
  const T * ptr;
};

template <typename T> constexpr named_value<T> nv(std::string_view name, const T & v)
{
  return {name, &v};
}

void log_info_line(std::string_view text);

template <typename... NVs> inline void log_info(std::string prefix, const NVs &... nvs)
{
  std::ostringstream oss;
  oss << std::boolalpha;
  ((oss << prefix << nvs.name << " = " << *(nvs.ptr) << ::color_text::RESET << '\n'), ...);
  log_info_line(oss.str());
}

// 带分割线与列对齐的块状调试输出
template <typename... NVs> inline void log_block(std::string prefix, const NVs &... nvs)
{
  // 计算最大键名宽度
  std::size_t maxw = 0;
  (void)std::initializer_list<int>{(maxw = std::max<std::size_t>(maxw, nvs.name.size()), 0)...};

  // 构造分割线（至少 50 列）
  const std::size_t bar_len = std::max<std::size_t>(50, maxw + 10);
  std::string bar(bar_len, '-');

  std::ostringstream oss;
  oss << std::boolalpha;
  // 时间戳 + 顶部分割线
  oss << prefix << '[' << now_string() << "] " << bar << ::color_text::RESET << '\n';
  // 对齐的 name : value
  ((oss << prefix << std::left << std::setw(static_cast<int>(maxw)) << nvs.name << " : " << *(nvs.ptr)
        << ::color_text::RESET << '\n'),
    ...);
  // 底部分割线
  oss << prefix << bar << ::color_text::RESET << '\n';

  log_info_line(oss.str());
}
}  // namespace custom_log

#define NV(var) ::custom_log::nv(#var, (var))
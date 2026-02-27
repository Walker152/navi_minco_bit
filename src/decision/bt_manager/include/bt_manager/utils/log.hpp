// Simplified logging utilities: free functions in a dedicated namespace to avoid
// conflicts with global math function "log" and allow easy NV(...) usage in macro argument lists.
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

namespace icp_log
{
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
    const T* ptr;
  };

  template <typename T> constexpr named_value<T> nv(std::string_view name, const T& v)
  {
    return {name, &v};
  }

  inline void log_info_line(std::string_view text)
  {
    std::cout << text << std::flush;
  }

  template <typename... NVs> inline void log_info(std::string prefix, const NVs&... nvs)
  {
    std::ostringstream oss;
    oss << std::boolalpha;
    ((oss << prefix << nvs.name << " = " << *(nvs.ptr) << ::color_text::RESET << '\n'), ...);
    log_info_line(oss.str());
  }

  template <typename... NVs> inline void log_block(std::string prefix, const NVs&... nvs)
  {
    std::size_t maxw = 0;
    (void)std::initializer_list<int>{(maxw = std::max<std::size_t>(maxw, nvs.name.size()), 0)...};

    const std::size_t bar_len = std::max<std::size_t>(35, maxw + 10);
    std::string bar(bar_len, '-');

    std::ostringstream oss;
    oss << std::boolalpha;
    oss << prefix << '[' << now_string() << "] " << bar << ::color_text::RESET << '\n';
    ((oss << prefix << std::left << std::setw(static_cast<int>(maxw)) << nvs.name << " : " << *(nvs.ptr)
          << ::color_text::RESET << '\n'),
     ...);
    oss << prefix << bar << ::color_text::RESET << '\n';
    log_info_line(oss.str());
  }
}  // namespace icp_log

#define NV(var) icp_log::nv(#var, (var))

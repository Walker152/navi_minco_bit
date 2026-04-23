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
#include <unordered_map>

namespace icp_log {
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

inline void log_info_line(std::string_view text)
{
  std::cout << text << std::flush;
}

template <typename... NVs> inline void log_info(std::string prefix, const NVs &... nvs)
{
  std::ostringstream oss;
  oss << std::boolalpha;
  ((oss << prefix << nvs.name << " = " << *(nvs.ptr) << ::color_text::RESET << '\n'), ...);
  log_info_line(oss.str());
}

template <typename... NVs> inline void log_block(std::string prefix, const NVs &... nvs)
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

namespace Sentry_BT {
namespace detail {
enum class TreeKind
{
  NAV,
  STANCE,
  GIMBAL
};

inline const std::string & treeColor(const TreeKind kind)
{
  switch (kind) {
    case TreeKind::NAV:
      return ::color_text::CYAN;
    case TreeKind::STANCE:
      return ::color_text::MAGENTA;
    case TreeKind::GIMBAL:
      return ::color_text::BLUE;
    default:
      return ::color_text::WHITE;
  }
}

inline const char * treeLabel(const TreeKind kind)
{
  switch (kind) {
    case TreeKind::NAV:
      return "NAV_TREE";
    case TreeKind::STANCE:
      return "STANCE_TREE";
    case TreeKind::GIMBAL:
      return "GIMBAL_TREE";
    default:
      return "BT_TREE";
  }
}

inline void logTransition(
  const TreeKind tree_kind, const std::string & condition_name, const bool active,
  const std::string & detail = "")
{
  static std::unordered_map<std::string, bool> last_states;
  const std::string key = std::string(treeLabel(tree_kind)) + "::" + condition_name;
  const auto it = last_states.find(key);
  if (it != last_states.end() && it->second == active) {
    return;
  }
  last_states[key] = active;

  std::cout << treeColor(tree_kind) << "[" << treeLabel(tree_kind) << "] " << condition_name << " => "
            << (active ? std::string(::color_text::GREEN) + "ACTIVE"
                       : std::string(::color_text::YELLOW) + "INACTIVE")
            << ::color_text::WHITE;
  if (!detail.empty()) {
    std::cout << " | " << detail;
  }
  std::cout << ::color_text::RESET << std::endl;
}
}  // namespace detail
}  // namespace Sentry_BT

#define NV(var) icp_log::nv(#var, (var))

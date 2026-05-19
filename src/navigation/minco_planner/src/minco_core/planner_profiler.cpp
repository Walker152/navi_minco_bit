#include "minco_core/planner_profiler.hpp"

#include <array>
#include <ctime>
#include <iomanip>
#include <sstream>

#include "nav2_util/node_utils.hpp"

namespace minco_planner {

namespace {

constexpr std::array<const char *, 10> kStageColumns = {
  "global_search_ms",
  "extract_local_path_ms",
  "sparsify_ms",
  "backup_traj_ms",
  "minco_opt_ms",
  "validate_ms",
  "yaw_opt_ms",
  "publish_visual_ms",
  "replan_local_total_ms",
  "safety_check_ms"};

constexpr std::array<const char *, 5> kMetricColumns = {
  "global_path_size",
  "dense_local_path_size",
  "sparse_path_size",
  "traj_duration",
  "final_cost"};

}  // namespace

PlannerProfiler::~PlannerProfiler()
{
  close();
}

void PlannerProfiler::configure(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & param_prefix,
  const rclcpp::Logger & logger)
{
  std::lock_guard<std::mutex> lock(mutex_);
  node_ = node;
  logger_ = logger;
  declareAndReadParameters(node, param_prefix);

  last_log_time_ = std::chrono::steady_clock::now();
  csv_header_written_ = false;
  csv_warned_ = false;

  if (!enabled_ || !csv_enabled_) {
    return;
  }

  csv_file_.open(csv_path_, std::ios::out | std::ios::app);
  if (!csv_file_.is_open()) {
    warnCsvFailure("failed to open " + csv_path_);
    return;
  }

  const auto pos = csv_file_.tellp();
  csv_header_written_ = (pos > std::ofstream::pos_type(0));
  writeHeaderIfNeeded();
}

void PlannerProfiler::declareAndReadParameters(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & prefix)
{
  auto node_ptr = node.lock();
  if (!node_ptr) {
    return;
  }

  nav2_util::declare_parameter_if_not_declared(
    node_ptr, prefix + "profiler.enabled", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_ptr, prefix + "profiler.log_enabled", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node_ptr, prefix + "profiler.csv_enabled", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_ptr, prefix + "profiler.log_period_sec", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_ptr, prefix + "profiler.csv_path", rclcpp::ParameterValue(std::string("/tmp/minco_planner_profile.csv")));

  node_ptr->get_parameter(prefix + "profiler.enabled", enabled_);
  node_ptr->get_parameter(prefix + "profiler.log_enabled", log_enabled_);
  node_ptr->get_parameter(prefix + "profiler.csv_enabled", csv_enabled_);
  node_ptr->get_parameter(prefix + "profiler.log_period_sec", log_period_sec_);
  node_ptr->get_parameter(prefix + "profiler.csv_path", csv_path_);

  if (log_period_sec_ < 0.1) {
    log_period_sec_ = 0.1;
  }
}

void PlannerProfiler::beginCycle(const std::string & event_name)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  current_ = CycleData{};
  current_.event = event_name;
  current_.start_time = std::chrono::steady_clock::now();
  current_.active = true;
}

bool PlannerProfiler::hasActiveCycle() const
{
  if (!enabled_) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  return current_.active;
}

void PlannerProfiler::mark(const std::string & stage_name, double duration_ms)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!current_.active) {
    return;
  }
  current_.stages[stage_name] = duration_ms;
}

void PlannerProfiler::setMetric(const std::string & metric_name, double value)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!current_.active) {
    return;
  }
  current_.metrics[metric_name] = value;
}

void PlannerProfiler::finishCycle(bool success)
{
  if (!enabled_) {
    return;
  }

  CycleData snapshot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!current_.active) {
      return;
    }
    snapshot = current_;
    current_.active = false;
  }

  maybeLogSummary(snapshot, success);
  writeCsvRow(snapshot, success);
}

void PlannerProfiler::recordSafetyCheck(double duration_ms, bool safe)
{
  if (!enabled_) {
    return;
  }

  CycleData data;
  data.event = "safety_check";
  data.active = false;
  data.stages["safety_check_ms"] = duration_ms;

  maybeLogSummary(data, safe);
  writeCsvRow(data, safe);
}

void PlannerProfiler::close()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (csv_file_.is_open()) {
    csv_file_.flush();
    csv_file_.close();
  }
}

void PlannerProfiler::writeHeaderIfNeeded()
{
  if (!csv_file_.is_open() || csv_header_written_) {
    return;
  }

  csv_file_ << "timestamp,event,success";
  for (const auto * column : kMetricColumns) {
    csv_file_ << ',' << column;
  }
  for (const auto * column : kStageColumns) {
    csv_file_ << ',' << column;
  }
  csv_file_ << '\n';
  csv_header_written_ = true;
}

void PlannerProfiler::writeCsvRow(const CycleData & data, bool success)
{
  if (!enabled_ || !csv_enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!csv_file_.is_open()) {
    warnCsvFailure("CSV file is not open");
    return;
  }

  writeHeaderIfNeeded();

  csv_file_ << wallTimeString() << ',' << data.event << ',' << (success ? 1 : 0);
  csv_file_ << std::fixed << std::setprecision(3);
  for (const auto * column : kMetricColumns) {
    csv_file_ << ',' << getValue(data.metrics, column);
  }
  for (const auto * column : kStageColumns) {
    csv_file_ << ',' << getValue(data.stages, column);
  }
  csv_file_ << '\n';

  if (!csv_file_) {
    warnCsvFailure("write failed");
    csv_file_.clear();
  }
}

void PlannerProfiler::maybeLogSummary(const CycleData & data, bool success)
{
  if (!enabled_ || !log_enabled_) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double elapsed = std::chrono::duration<double>(now - last_log_time_).count();
    if (elapsed < log_period_sec_) {
      return;
    }
    last_log_time_ = now;
  }

  RCLCPP_INFO(logger_,
    "[PlannerProfiler] event=%s success=%s total=%.3fms global=%.3fms opt=%.3fms validate=%.3fms "
    "safety=%.3fms path=%0.f/%0.f/%0.f cost=%.3f",
    data.event.c_str(),
    success ? "true" : "false",
    getValue(data.stages, "replan_local_total_ms"),
    getValue(data.stages, "global_search_ms"),
    getValue(data.stages, "minco_opt_ms"),
    getValue(data.stages, "validate_ms"),
    getValue(data.stages, "safety_check_ms"),
    getValue(data.metrics, "global_path_size"),
    getValue(data.metrics, "dense_local_path_size"),
    getValue(data.metrics, "sparse_path_size"),
    getValue(data.metrics, "final_cost"));
}

void PlannerProfiler::warnCsvFailure(const std::string & reason)
{
  auto node = node_.lock();
  if (node) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), 5000, "[PlannerProfiler] CSV output disabled or failed: %s", reason.c_str());
    return;
  }

  if (!csv_warned_) {
    RCLCPP_WARN(logger_, "[PlannerProfiler] CSV output disabled or failed: %s", reason.c_str());
    csv_warned_ = true;
  }
}

std::string PlannerProfiler::wallTimeString()
{
  using namespace std::chrono;
  const auto now = system_clock::now();
  const auto now_time = system_clock::to_time_t(now);
  std::tm tm_buf{};
#if defined(_WIN32)
  localtime_s(&tm_buf, &now_time);
#else
  localtime_r(&now_time, &tm_buf);
#endif
  const auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

  std::ostringstream oss;
  oss << std::setfill('0') << std::setw(4) << (tm_buf.tm_year + 1900) << '-' << std::setw(2)
      << (tm_buf.tm_mon + 1) << '-' << std::setw(2) << tm_buf.tm_mday << ' ' << std::setw(2)
      << tm_buf.tm_hour << ':' << std::setw(2) << tm_buf.tm_min << ':' << std::setw(2) << tm_buf.tm_sec
      << '.' << std::setw(3) << ms.count();
  return oss.str();
}

double PlannerProfiler::getValue(
  const std::unordered_map<std::string, double> & values,
  const std::string & key)
{
  const auto it = values.find(key);
  return it == values.end() ? 0.0 : it->second;
}

}  // namespace minco_planner

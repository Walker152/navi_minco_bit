#include "bt_manager/behavior_tree_logger.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace Sentry_BT {

TeeStreamBuffer::TeeStreamBuffer(
  std::streambuf * terminal_buffer, std::ofstream & log_file, std::mutex & mutex)
: terminal_buffer_(terminal_buffer), log_file_(log_file), mutex_(mutex)
{
}

TeeStreamBuffer::int_type TeeStreamBuffer::overflow(const int_type character)
{
  if (traits_type::eq_int_type(character, traits_type::eof())) {
    return traits_type::not_eof(character);
  }
  const char value = traits_type::to_char_type(character);
  return xsputn(&value, 1) == 1 ? character : traits_type::eof();
}

std::streamsize TeeStreamBuffer::xsputn(const char * text, const std::streamsize size)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto terminal_written = terminal_buffer_->sputn(text, size);
  for (std::streamsize index = 0; index < size; ++index) {
    stripAnsiAndWrite(text[index]);
  }
  return terminal_written;
}

int TeeStreamBuffer::sync()
{
  std::lock_guard<std::mutex> lock(mutex_);
  log_file_.flush();
  return terminal_buffer_->pubsync();
}

void TeeStreamBuffer::stripAnsiAndWrite(const char character)
{
  // Strip ANSI color/control sequences from the persistent log.
  if (ansi_state_ == 0 && character == '\x1b') {
    ansi_state_ = 1;
    return;
  }
  if (ansi_state_ == 1) {
    ansi_state_ = (character == '[') ? 2 : 0;
    return;
  }
  if (ansi_state_ == 2) {
    if (character >= '@' && character <= '~') {
      ansi_state_ = 0;
    }
    return;
  }
  log_file_.put(character);
}

BehaviorTreeLogSink::BehaviorTreeLogSink()
{
  file_path_ = makeLogFilePath();
  const std::filesystem::path path(file_path_);
  std::filesystem::create_directories(path.parent_path());
  log_file_.open(path, std::ios::out | std::ios::app);
  if (!log_file_.is_open()) {
    throw std::runtime_error("Failed to open behavior-tree log file: " + file_path_);
  }

  original_cout_buffer_ = std::cout.rdbuf();
  tee_buffer_ = std::make_unique<TeeStreamBuffer>(original_cout_buffer_, log_file_, mutex_);
  std::cout.rdbuf(tee_buffer_.get());
  std::cout << "[BT_LOGGER] Logging behavior-tree output to " << file_path_ << std::endl;
}

BehaviorTreeLogSink::~BehaviorTreeLogSink()
{
  if (original_cout_buffer_ != nullptr) {
    std::cout.flush();
    std::cout.rdbuf(original_cout_buffer_);
  }
}

std::string BehaviorTreeLogSink::makeLogFilePath()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
  localtime_r(&now_time, &local_time);

  std::ostringstream filename;
  filename << "bt_" << std::put_time(&local_time, "%Y%m%d_%H%M%S") << ".log";
  return (std::filesystem::current_path() / "docs/bt_logs" / filename.str()).string();
}

BehaviorTreeLogger::BehaviorTreeLogger(const BT::Tree & tree, std::string tree_label)
: BT::StatusChangeLogger(tree.rootNode()), tree_label_(std::move(tree_label))
{
  setTimestampType(BT::TimestampType::relative);
}

void BehaviorTreeLogger::callback(BT::Duration timestamp,
  const BT::TreeNode & node,
  const BT::NodeStatus previous_status,
  const BT::NodeStatus status)
{
  const auto seconds = std::chrono::duration<double>(timestamp).count();
  std::ostringstream line;
  line << std::fixed << std::setprecision(3) << "[BT_STATUS][" << tree_label_ << "][" << seconds
       << "s] " << node.name() << " " << BT::toStr(previous_status, false) << " -> "
       << BT::toStr(status, false);
  std::cout << line.str() << std::endl;
}

void BehaviorTreeLogger::flush()
{
  std::cout.flush();
}

}  // namespace Sentry_BT

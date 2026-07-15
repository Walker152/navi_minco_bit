#pragma once

#include <fstream>
#include <memory>
#include <mutex>
#include <ostream>
#include <streambuf>
#include <string>

#include <behaviortree_cpp_v3/loggers/abstract_logger.h>

namespace Sentry_BT {

class TeeStreamBuffer : public std::streambuf
{
public:
  TeeStreamBuffer(std::streambuf * terminal_buffer, std::ofstream & log_file, std::mutex & mutex);

protected:
  int_type overflow(int_type character) override;
  std::streamsize xsputn(const char * text, std::streamsize size) override;
  int sync() override;

private:
  void stripAnsiAndWrite(char character);

  std::streambuf * terminal_buffer_;
  std::ofstream & log_file_;
  std::mutex & mutex_;
  int ansi_state_{0};
};

class BehaviorTreeLogSink
{
public:
  BehaviorTreeLogSink();
  ~BehaviorTreeLogSink();

  BehaviorTreeLogSink(const BehaviorTreeLogSink &) = delete;
  BehaviorTreeLogSink & operator=(const BehaviorTreeLogSink &) = delete;

  const std::string & filePath() const { return file_path_; }

private:
  static std::string makeLogFilePath();

  std::mutex mutex_;
  std::ofstream log_file_;
  std::streambuf * original_cout_buffer_{nullptr};
  std::unique_ptr<TeeStreamBuffer> tee_buffer_;
  std::string file_path_;
};

class BehaviorTreeLogger : public BT::StatusChangeLogger
{
public:
  BehaviorTreeLogger(const BT::Tree & tree, std::string tree_label);

  void callback(BT::Duration timestamp,
    const BT::TreeNode & node,
    BT::NodeStatus previous_status,
    BT::NodeStatus status) override;
  void flush() override;

private:
  std::string tree_label_;
};

}  // namespace Sentry_BT

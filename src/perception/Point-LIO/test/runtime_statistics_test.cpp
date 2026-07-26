#include "runtime_statistics.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace {

std::filesystem::path make_test_directory(const std::string & suffix)
{
  const auto token = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
         ("point_lio_runtime_statistics_" + suffix + "_" + std::to_string(token));
}

std::string read_file(const std::filesystem::path & path)
{
  std::ifstream stream(path);
  return std::string(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
}

std::size_t whitespace_token_count(const std::string & text)
{
  std::istringstream stream(text);
  std::size_t count = 0;
  std::string token;
  while (stream >> token) {
    ++count;
  }
  return count;
}

}  // namespace

TEST(RuntimeStatistics, DisabledModeDoesNotCreateLogFiles)
{
  const auto directory = make_test_directory("disabled");
  point_lio::RuntimeStatistics statistics;

  EXPECT_TRUE(statistics.initialize(false, directory, false, false, 1.0));
  EXPECT_FALSE(statistics.enabled());
  EXPECT_FALSE(std::filesystem::exists(directory));
}

TEST(RuntimeStatistics, ResolvesConfiguredLogDirectory)
{
  const std::filesystem::path root = "/tmp/point_lio_root";

  EXPECT_EQ(point_lio::resolveLogDirectory("", root), root / "Log");
  EXPECT_EQ(point_lio::resolveLogDirectory("vehicle_logs", root), root / "vehicle_logs");
  EXPECT_EQ(
    point_lio::resolveLogDirectory("/data/logpointlio", root), std::filesystem::path("/data/logpointlio"));
}

TEST(RuntimeStatistics, EnabledModeOwnsLegacyAndPerformanceLogs)
{
  const auto directory = make_test_directory("enabled");
  point_lio::RuntimeStatistics statistics;

  ASSERT_TRUE(statistics.initialize(true, directory, false, false, 1.0));
  EXPECT_TRUE(statistics.enabled());

  point_lio::ImuLogRecord imu;
  imu.sequence = 1;
  imu.original_stamp = 10.0;
  imu.corrected_stamp = 10.0;
  statistics.recordImu(imu);

  point_lio::StateLogRecord state;
  state.relative_time = 1.0;
  statistics.recordMat(state);
  statistics.recordPos(state);

  point_lio::FramePerformanceRecord frame;
  frame.lidar_beg_stamp = 10.0;
  frame.lidar_end_stamp = 10.05;
  frame.latest_lidar_stamp = 10.10;
  frame.process_ms = 40.0;
  frame.effective_feature_points = 55;
  frame.ivox.sampled = true;
  frame.ivox.valid_grids = 120;
  frame.ivox.points_per_grid_avg = 8.5;
  frame.ivox.points_per_grid_max = 32;
  frame.ivox.points_per_grid_stddev = 4.25;
  frame.ivox.collection_ms = 0.2;
  statistics.recordPoseUpdate(80, 10.01, 2.5, true);
  statistics.recordPoseUpdate(70, 10.02, 1.5, false);
  statistics.recordFrame(frame);
  statistics.shutdown();

  EXPECT_TRUE(std::filesystem::exists(directory / "imu_pbp.txt"));
  EXPECT_TRUE(std::filesystem::exists(directory / "mat_out.txt"));
  EXPECT_TRUE(std::filesystem::exists(directory / "pos_log.txt"));
  EXPECT_TRUE(std::filesystem::exists(directory / "performance.csv"));

  EXPECT_NE(read_file(directory / "imu_pbp.txt").find("corrected_stamp"), std::string::npos);
  EXPECT_EQ(whitespace_token_count(read_file(directory / "mat_out.txt")), 26U);
  EXPECT_EQ(whitespace_token_count(read_file(directory / "pos_log.txt")), 25U);

  std::istringstream performance(read_file(directory / "performance.csv"));
  std::string header;
  std::string row;
  ASSERT_TRUE(static_cast<bool>(std::getline(performance, header)));
  ASSERT_TRUE(static_cast<bool>(std::getline(performance, row)));
  EXPECT_NE(header.find("lidar_backlog_ms"), std::string::npos);
  EXPECT_NE(header.find("pose_update_attempt_count"), std::string::npos);
  EXPECT_NE(header.find("pose_update_failure_count"), std::string::npos);
  EXPECT_NE(header.find("pose_update_points_avg"), std::string::npos);
  EXPECT_NE(header.find("ekf_update_ms"), std::string::npos);
  EXPECT_NE(header.find("effective_feature_points"), std::string::npos);
  EXPECT_NE(header.find("ivox_stats_sampled"), std::string::npos);
  EXPECT_NE(header.find("ivox_points_per_grid_avg"), std::string::npos);
  EXPECT_NE(header.find("ivox_stats_ms"), std::string::npos);
  EXPECT_EQ(std::count(header.begin(), header.end(), ','), 40);
  EXPECT_EQ(std::count(row.begin(), row.end(), ','), 40);

  std::filesystem::remove_all(directory);
}

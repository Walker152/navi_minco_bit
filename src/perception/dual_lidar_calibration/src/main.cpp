#include "dual_lidar_calibration/calibration_config.hpp"
#include "dual_lidar_calibration/offline_calibrator.hpp"
#include "dual_lidar_calibration/result_writer.hpp"

#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

struct Arguments
{
  std::string bag_path;
  std::string static_bag_path;
  std::string config_path;
  std::string output_path;
};

void printUsage(const char * executable)
{
  std::cerr << "Usage: " << executable
            << " --bag <rosbag2_path> [--static-bag <rosbag2_path>]"
               " --config <yaml_path> --output <directory>\n";
}

Arguments parseArguments(const int argc, char ** argv)
{
  Arguments arguments;
  for (int i = 1; i < argc; ++i) {
    const std::string option(argv[i]);
    if ((option == "--bag" || option == "--static-bag" || option == "--config" ||
          option == "--output") &&
        i + 1 < argc) {
      const std::string value(argv[++i]);
      if (option == "--bag") {
        arguments.bag_path = value;
      } else if (option == "--static-bag") {
        arguments.static_bag_path = value;
      } else if (option == "--config") {
        arguments.config_path = value;
      } else {
        arguments.output_path = value;
      }
      continue;
    }
    if (option == "--ros-args") {
      break;
    }
    throw std::invalid_argument("Unknown or incomplete command-line option: " + option);
  }
  if (arguments.bag_path.empty() || arguments.config_path.empty() || arguments.output_path.empty()) {
    throw std::invalid_argument("--bag, --config, and --output are all required");
  }
  return arguments;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Arguments arguments = parseArguments(argc, argv);
    rclcpp::init(argc, argv);
    const auto config = dual_lidar_calibration::loadCalibrationConfig(arguments.config_path);
    std::cout << "[dual_lidar_calibration] Reading bag and calibrating...\n";
    const auto result = dual_lidar_calibration::runOfflineCalibration(
      arguments.bag_path, config, arguments.static_bag_path);
    dual_lidar_calibration::writeCalibrationResults(arguments.output_path, config, result);
    std::cout << "[dual_lidar_calibration] " << result.message << '\n';
    std::cout << "[dual_lidar_calibration] synchronized_pairs=" << result.synchronized_pairs
              << " accepted_frames=" << result.aggregate.used_frames
              << " output=" << arguments.output_path << '\n';
    rclcpp::shutdown();
    return result.success ? 0 : 2;
  } catch (const std::exception & error) {
    std::cerr << "[dual_lidar_calibration] ERROR: " << error.what() << '\n';
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    printUsage(argv[0]);
    return 1;
  }
}

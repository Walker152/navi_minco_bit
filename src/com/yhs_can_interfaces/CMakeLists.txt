cmake_minimum_required(VERSION 3.8)
project(yhs_can_interfaces)

if(CMAKECOMPILER_IS_GNUCXX OR CMAKECXXCOMPILER_ID MATCHES "Clang")
  addCompile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BmsFlagFb.msg"
	"msg/BmsFb.msg"
	"msg/ChassisInfoFb.msg"
	"msg/CtrlCmd.msg"
	"msg/CtrlFb.msg"
	"msg/SteeringCtrlCmd.msg"
	"msg/SteeringCtrlFb.msg"
	"msg/FrontAngleFb.msg"
	"msg/IoCmd.msg"
	"msg/IoFb.msg"
	"msg/LrWheelFb.msg"
	"msg/RrWheelFb.msg"
	"msg/LfWheelFb.msg"
	"msg/RfWheelFb.msg"
	"msg/RearAngleFb.msg"
	"msg/Ultrasonic.msg"
	"msg/ErrorFb.msg"
	"msg/MotorCmd.msg"
	"msg/MotorFb.msg"
	"msg/DriveMotorCurrentFb.msg"
	"msg/SteeringMotorCurrentFb.msg"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(amentCmakeCopyrightFOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(amentCmakeCpplintFOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

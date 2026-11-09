cmake_minimum_required(VERSION 3.8)
project(driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
     "msg/BatteryData.msg"
     "msg/BatteriesData.msg"
     "msg/ChassisDeviceInfo.msg"
     "msg/DeviceInfo.msg"
     "msg/EulerAngles.msg"
     "msg/ImuData.msg"
     "msg/IoCmdData.msg"
     "msg/IoData.msg"
     "msg/JoystickData.msg"
     "msg/LightCmdData.msg"
     "msg/MotorCmdData.msg"
     "msg/MotorData.msg"
     "msg/MotorsData.msg"
     "msg/PowerData.msg"
     "msg/RadarData.msg"
     "msg/RadarsData.msg"
     "msg/RadarPoints.msg"
     "msg/SetMotorCmd.msg"
     "msg/SetShaftMotorCmd.msg"
     "msg/Stm32Data.msg"
     "msg/State.msg"
     "msg/CameraInfo.msg"
     "msg/CamerasInfo.msg"
     "msg/Stereo.msg"
     "msg/ShaftStateData.msg"
     "srv/ControlPower.srv"
     "srv/GetChassisDeviceInfo.srv"
     "srv/Reset.srv"
     "srv/SetIoOutput.srv"
     "srv/SetLight.srv"
     "srv/SetScram.srv"
     "srv/GetCameraInfo.srv"
     "srv/ShaftZeroMarking.srv"
      DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME})


ament_package()

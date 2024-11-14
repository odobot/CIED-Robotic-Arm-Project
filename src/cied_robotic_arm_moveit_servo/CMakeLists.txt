cmake_minimum_required(VERSION 3.8)
project(cied_robotic_arm_moveit_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/cmake/libusb-1.0" ${CMAKE_PREFIX_PATH})

# find dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(PkgConfig REQUIRED)


pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
include_directories(${LIBUSB_INCLUDE_DIRS})
link_directories(${LIBUSB_LIBRARY_DIRS})


set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  sensor_msgs
  control_msgs
  moveit_msgs
  geometry_msgs
  moveit_ros_planning
)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)


add_executable(
  keyboard_test_node src/keyboard_test.cpp
)
ament_target_dependencies(
  keyboard_test_node
  "rclcpp"
)
add_executable(
  joy_test_node src/joy_test.cpp
)
ament_target_dependencies(
  joy_test_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "control_msgs"
)
target_link_libraries(joy_test_node ${LIBUSB_LIBRARIES})

add_executable(cied_robotic_arm_keyboard_input
  src/cied_robotic_arm_keyboard_input.cpp
)
ament_target_dependencies(cied_robotic_arm_keyboard_input ${THIS_PACKAGE_INCLUDE_DEPENDS})


add_library(cied_robotic_arm_joystick_input SHARED src/cied_robotic_arm_joystick_input.cpp)
target_include_directories(
  cied_robotic_arm_joystick_input
  PRIVATE "cied_robotic_arm_joystick_input_node_CPP_BUILD_DLL"
)
ament_target_dependencies(cied_robotic_arm_joystick_input ${THIS_PACKAGE_INCLUDE_DEPENDS})

rclcpp_components_register_node(
  cied_robotic_arm_joystick_input
    PLUGIN "cied_robotic_arm_moveit_servo::JoyToServoPub"
    EXECUTABLE cied_robotic_arm_joystick_input_node
)

install(
  TARGETS 
    keyboard_test_node
    joy_test_node
  DESTINATION 
    lib/${PROJECT_NAME}
)

install(
  TARGETS
    cied_robotic_arm_joystick_input
    cied_robotic_arm_keyboard_input
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(
  DIRECTORY
    config
    launch
    rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

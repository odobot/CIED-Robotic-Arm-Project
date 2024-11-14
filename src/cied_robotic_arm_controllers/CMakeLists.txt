cmake_minimum_required(VERSION 3.8)
project(cied_robotic_arm_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(SERIAL libserial)

add_library(
  ${PROJECT_NAME} SHARED
  src/cied_robotic_arm_hardware.cpp
)

target_include_directories(
  ${PROJECT_NAME} PRIVATE
  include
  "${SERIAL_INCLUDE_DIRS}"
)

target_link_libraries(
  ${PROJECT_NAME}
  "${SERIAL_LDFLAGS}"
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp_lifecycle
  rclcpp
  pluginlib
  hardware_interface
)

pluginlib_export_plugin_description_file(hardware_interface cied_robotic_arm_hardware_plugin.xml)


install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include
  DESTINATION include
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
)


# Move the added folders to the share folder of the project
install(
  DIRECTORY 
    launch config
  DESTINATION 
    share/${PROJECT_NAME}
)

ament_package()

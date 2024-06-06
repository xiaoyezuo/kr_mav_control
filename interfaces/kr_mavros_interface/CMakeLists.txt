cmake_minimum_required(VERSION 3.10)
project(kr_mavros_interface)

# set default build type
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kr_mav_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# If mavros_msgs not found, don't build but do not cause a build error
find_package(mavros_msgs)

# if(NOT ${mavros_msgs_FOUND})
#   message(WARNING "NOTE: mavros_msgs not found so not building kr_mavros_interface")
# else()

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/so3cmd_to_mavros_component.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_components kr_mav_msgs nav_msgs sensor_msgs geometry_msgs tf2 tf2_geometry_msgs mavros_msgs)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
rclcpp_components_register_nodes(${PROJECT_NAME} "SO3CmdToMavros")

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  include_directories(include)
  ament_add_gtest_executable(so3_to_mavros_component_test test/so3cmd_to_mavros_test.cpp)
  ament_target_dependencies(so3_to_mavros_component_test
    std_msgs
    kr_mav_msgs
    nav_msgs
    rclcpp
    rclcpp_components
    sensor_msgs
    geometry_msgs
    Eigen3
    tf2
    tf2_geometry_msgs
    mavros_msgs
  )

  add_launch_test(launch/so3cmd_to_mavros_test.test.py TIMEOUT 80 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

endif()

ament_export_dependencies(rclcpp rclcpp_components kr_mav_msgs nav_msgs geometry_msgs Eigen3 sensor_msgs tf2 tf2_geometry_msgs mavros_msgs)
ament_package()

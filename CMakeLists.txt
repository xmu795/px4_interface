cmake_minimum_required(VERSION 3.8)
project(px4_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleStatus.msg"
  "msg/PositionNED.msg"
  "msg/BatteryStatus.msg"
  DEPENDENCIES builtin_interfaces
)

# include directories
include_directories(include ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

# Create the px4_msgs_cache library
add_library(px4_msgs_cache src/px4_msgs_cache.cpp)
target_include_directories(px4_msgs_cache PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR} # Eigen3 include directory
)
target_link_libraries(px4_msgs_cache ${rclcpp_LIBRARIES})
ament_target_dependencies(px4_msgs_cache rclcpp std_msgs Eigen3 rosidl_default_runtime)

# Create the px4_gateway library
add_library(px4_gateway src/px4_gateway.cpp)
target_include_directories(px4_gateway PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR} # Eigen3 include directory
)
target_link_libraries(px4_gateway px4_msgs_cache ${rclcpp_LIBRARIES})
ament_target_dependencies(px4_gateway rclcpp std_msgs Eigen3 px4_msgs rosidl_default_runtime)

# Install targets
install(TARGETS px4_msgs_cache px4_gateway
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  # Add unit tests
  ament_add_gtest(test_px4_msgs_cache test/test_px4_msgs_cache.cpp)
  target_include_directories(test_px4_msgs_cache PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_px4_msgs_cache px4_msgs_cache)
  ament_target_dependencies(test_px4_msgs_cache rclcpp std_msgs Eigen3)
endif()

ament_package()

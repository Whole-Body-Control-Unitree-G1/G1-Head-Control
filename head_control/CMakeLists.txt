cmake_minimum_required(VERSION 3.20)
project(head_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)

# Shared driver library
add_library(dynamixel_driver src/dynamixel_driver.cpp)
target_link_libraries(dynamixel_driver PUBLIC
  dynamixel_sdk::dynamixel_sdk
)
target_include_directories(dynamixel_driver PUBLIC include)

# head_control_node
add_executable(head_control_node src/head_control_node.cpp)
target_link_libraries(head_control_node PRIVATE
  dynamixel_driver
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)

# calibration_node
add_executable(calibration_node src/calibration_node.cpp)
target_link_libraries(calibration_node PRIVATE
  dynamixel_driver
  rclcpp::rclcpp
  yaml-cpp
  ament_index_cpp::ament_index_cpp
)

install(TARGETS dynamixel_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  head_control_node
  calibration_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install other config files normally, but symlink calib.yaml back to the source
# so that calibration writes persist across builds (writing to the install path
# goes through the symlink and updates the source file directly).
install(CODE "
  file(MAKE_DIRECTORY \"\${CMAKE_INSTALL_PREFIX}/share/head_control/config\")
  file(CREATE_LINK
    \"${CMAKE_CURRENT_SOURCE_DIR}/config/calib.yaml\"
    \"\${CMAKE_INSTALL_PREFIX}/share/head_control/config/calib.yaml\"
    SYMBOLIC
  )
")

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(rov_pid_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ControllerStatus.msg
  srv/SetAxisMode.srv
  srv/SetAllModes.srv
  srv/CaptureSetpoint.srv
  DEPENDENCIES std_msgs
)

# --- Core PID/axis library ---
add_library(rov_pid_core
  src/pid.cpp
  src/cascaded_axis.cpp
)
target_include_directories(rov_pid_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(rov_pid_core PUBLIC cxx_std_17)

# --- Controller node ---
add_executable(rov_pid_controller_node
  src/controller_node.cpp
  src/controller_main.cpp
)
target_include_directories(rov_pid_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rov_pid_controller_node rov_pid_core)
ament_target_dependencies(rov_pid_controller_node
  rclcpp geometry_msgs nav_msgs std_msgs std_srvs tf2 tf2_geometry_msgs
)

# Link generated interfaces into the node
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(rov_pid_controller_node "${cpp_typesupport_target}")

# --- Install ---
install(TARGETS rov_pid_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS rov_pid_controller_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY foxglove
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# --- Python control panel ---
install(PROGRAMS scripts/axis_panel.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME axis_panel
)

ament_export_include_directories(include)
ament_package()

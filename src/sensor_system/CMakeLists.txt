cmake_minimum_required(VERSION 3.8)
project(sensor_system)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 1. Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2. Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorSignal.msg"
  DEPENDENCIES builtin_interfaces
)

# 3. Sensor Node 1
add_executable(sensor_node_1 src/sensor_node_1.cpp)
# This is the critical line missing - it adds the rclcpp headers
ament_target_dependencies(sensor_node_1 rclcpp)

# Sensor Node 2
add_executable(sensor_node_2 src/sensor_node_2.cpp)
ament_target_dependencies(sensor_node_2 rclcpp)
# Link the generated typesupport libraries


rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(sensor_node_1 "${cpp_typesupport_target}")

target_link_libraries(sensor_node_2 "${cpp_typesupport_target}")

# 4. Central Node
add_executable(central_node src/central_node.cpp)
# This adds the rclcpp headers to central_node
ament_target_dependencies(central_node rclcpp)
target_link_libraries(central_node "${cpp_typesupport_target}")

# 5. Install
install(TARGETS sensor_node_1 sensor_node_2 central_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
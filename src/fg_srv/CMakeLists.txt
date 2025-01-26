cmake_minimum_required(VERSION 3.8)
project(fg_srv)

# 1. Find core dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2. Generate code for your custom action
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Mission.action"
  DEPENDENCIES builtin_interfaces
)

# 3. Install your Python package so ROS can import it (fg_srv.*)
ament_python_install_package(${PROJECT_NAME})

# 4. Export runtime dependencies for the generated actions
ament_export_dependencies(rosidl_default_runtime)

# 5. Install launch files to allow 'ros2 launch fg_srv <launch_file>'
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 6. Finalize the ament package
ament_package()

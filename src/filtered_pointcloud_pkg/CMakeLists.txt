cmake_minimum_required(VERSION 3.5)
project(filtered_pointcloud_pkg)  # 🔄 Make sure this matches the actual package name

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the service interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FilterLimits.srv"
)

# === For Python nodes ===
ament_python_install_package(${PROJECT_NAME})

# === Install Python modules and scripts ===
install(
  DIRECTORY ${PROJECT_NAME}
  DESTINATION lib/python3.12/site-packages  # or just 'lib' if unsure
)

install(PROGRAMS
  ${PROJECT_NAME}/threshold_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

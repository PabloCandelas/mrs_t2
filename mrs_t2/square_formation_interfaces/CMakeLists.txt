cmake_minimum_required(VERSION 3.8)
project(square_formation_interfaces)

# -----------------------------
# Dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_generator_py REQUIRED)

# -----------------------------
# Generate actions
# -----------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveTo.action"
  DEPENDENCIES geometry_msgs action_msgs
)

# -----------------------------
# Export dependencies
# -----------------------------
ament_export_dependencies(rosidl_default_runtime)

# -----------------------------
# Finalize package
# -----------------------------
ament_package()
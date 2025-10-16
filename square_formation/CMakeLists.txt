cmake_minimum_required(VERSION 3.8)
project(square_formation)

# -----------------------------
# Dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(square_formation_interfaces REQUIRED)  # Your interface package

# -----------------------------
# Python package installation
# -----------------------------
ament_python_install_package(${PROJECT_NAME})

# -----------------------------
# Export dependencies
# -----------------------------
ament_export_dependencies(rclpy square_formation_interfaces)

# -----------------------------
# Finalize package
# -----------------------------
ament_package()

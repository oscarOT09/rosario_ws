cmake_minimum_required(VERSION 3.8)
project(action_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encuentra dependencias necesarias
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generar interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/YoloAction.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

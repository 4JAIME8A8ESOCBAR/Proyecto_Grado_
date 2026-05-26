cmake_minimum_required(VERSION 3.8)
project(mi_robot_rescate)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Buscar dependencias de ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# --- INSTALACIÓN DE LOS SCRIPTS DE PYTHON ---
# Esto copia tus scripts a la carpeta de ejecutables de ROS 2
install(PROGRAMS
  mi_robot_rescate/detector_rescate.py
  mi_robot_rescate/filtro_humo.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- INSTALACIÓN DE ARCHIVOS DE CONFIGURACIÓN Y LAUNCH ---
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
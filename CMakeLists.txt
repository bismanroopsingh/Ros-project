cmake_minimum_required(VERSION 3.8)
project(hospital_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes worlds launch config maps
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/odom_tf_publisher.py
    scripts/pd_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
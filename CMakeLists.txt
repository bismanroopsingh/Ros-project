cmake_minimum_required(VERSION 3.8)
project(hospital_robot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config urdf worlds maps meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/odom_tf_publisher.py
    scripts/pd_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/cmd_vel_relay.py
  scripts/recovery_helper.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
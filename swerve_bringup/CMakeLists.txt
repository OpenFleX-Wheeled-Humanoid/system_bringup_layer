cmake_minimum_required(VERSION 3.8)
project(swerve_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/swerve_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/vr_teleop_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME vr_teleop_node
)

install(
  PROGRAMS scripts/vr_lift_control_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME vr_lift_control_node
)

install(
  PROGRAMS scripts/waist_chassis_control_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME waist_chassis_control_node
)

install(
  PROGRAMS scripts/vr_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME vr_monitor
)

ament_package()

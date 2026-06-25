cmake_minimum_required(VERSION 3.8)
project(franka_pose_estimation)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetObjectPose.srv"
  DEPENDENCIES geometry_msgs
)

# The node is a plain Python script (no rosidl deps at build time); it imports
# the generated franka_pose_estimation.srv package at runtime.
install(PROGRAMS
  scripts/foundationpose_pose_service.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

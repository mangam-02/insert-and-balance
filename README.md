# Manipulatoren

Status: Position Planning mit Moveit geht, 
    gripper muss noch hinyugefugt werden
    impedance control
    frame verstehen und zu posen verfahren





## Usefull comands

- Launch the Robot with RVIZ and Position Control in Moveit:

ros2 launch franka_moveit_config moveit.launch.py robot_ip:=172.16.0.2 use_rviz:=true


- Launch Docker container
cd manipulatoren/multipanda_ros2
./run

- Enter Docker Container 
newgrp docker
docker exec -it --user developer multipanda-container bash



## Gripper Usage

  source /opt/ros/humble/setup.bash
  source ~/multipanda_ws/install/setup.bash
  ros2 launch franka_gripper gripper.launch.py robot_ip:=172.16.0.2 arm_id:=panda

  Schritt 2 — in einem Terminal mit gesourctem Workspace ansteuern:
  source ~/multipanda_ws/install/setup.bash      # <-- wichtig, sonst "invalid action type"

  # Kalibrieren (zuerst):
  ros2 action send_goal /panda_gripper/homing franka_msgs/action/Homing "{}"
  
  # Öffnen:
  ros2 action send_goal /panda_gripper/move franka_msgs/action/Move "{width: 0.08, speed: 0.1}"

  # Schließen:
  ros2 action send_goal /panda_gripper/move franka_msgs/action/Move "{width: 0.0, speed: 0.1}"
  
  
  
  
  
  
  # Backup Extrinsics
  
developer@minga-09:~/wrist_cam_calibration/run1$ cat ~/wrist_cam_calibration/run1/wrist_camera_extrinsics.yaml
# Eye-in-hand wrist-camera extrinsics.
# Transform: camera optical frame expressed in "panda_link8".
# Method: cv2.calibrateHandEye / park, 6 samples.
wrist_camera_extrinsics:
  parent_frame: panda_link8
  child_frame: camera_optical_frame
  translation:
    x: 0.00790127
    y: -0.08703501
    z: 0.05698960
  rotation_quaternion_xyzw:
    x: 0.00827584
    y: 0.30689029
    z: 0.95060955
    w: 0.04573117
  consistency_residual: 0.00929430
# Static TF example:
#   ros2 run tf2_ros static_transform_publisher 0.007901 -0.087035 0.056990 0.008276 0.306890 0.950610 0.045731 panda_link8 camera_optical_frame
developer@minga-09:~/wrist_cam_calibration/run1$ 



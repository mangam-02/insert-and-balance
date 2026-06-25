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

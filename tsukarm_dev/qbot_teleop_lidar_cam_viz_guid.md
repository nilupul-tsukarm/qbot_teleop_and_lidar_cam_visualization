these implementations done in inside the qbot AMR nvidia pc 

/home/nvidia/tsukarm_dev <--------------- for robot AMR motions  

/home/nvidia/ros2_ws  <--------------- for robot Lidar and Camera wakeup and publish 

foxglove <--------------- for remote obseve the publish data over local network 

qbot AMR nvidia pc and conroller pc (windows) connect to same local network same as ssh 

for wakeup Lidar and realsense camera (simple build at 1st time copy the "qbot_sensors"package to "..ros2_ws/src/" . colocn build at "..ros2_ws/")

    cd ros2_ws/
    source install/setup.bash
    source /opt/ros/humble/setup.bash
    ros2 launch qbot_sensors sensors.launch.py

to start foxglove from qbot AMR nvidia pc side 

    ros2 run foxglove_bridge foxglove_bridge

then connect Enter ws://<ROBOT_IP_ADDRESS>:8765 (replace <ROBOT_IP_ADDRESS> with the IP you use to SSH into the robot), and add "/camera/color/image_raw" , For the LiDAR: Add a 3D panel. In the settings on the left sidebar, toggle /scan to visible. Ensure the Global Frame is set to base_link or lidar

for contol robot AMR motions 
    cd /home/nvidia/tsukarm_dev
    python3 motion_with_keyboard.py


----------QBot AMR Setup and Operation Guide------------

These implementations are executed directly on the QBot AMR NVIDIA PC. Ensure both the QBot NVIDIA PC and your Windows controller PC are connected to the same local network.

Directory Structure

    /home/nvidia/tsukarm_dev — Scripts for controlling robot AMR motions.

    /home/nvidia/ros2_ws — Workspace for waking up and publishing LiDAR and RealSense camera data.

    Foxglove — Used on the Windows controller PC to remotely observe published sensor data.

1. Launch Sensors (LiDAR & RealSense Camera)

First-Time Setup (Build):
If this is your first time, copy the qbot_sensors package into the src directory and build the workspace.
    
    cd ~/ros2_ws/
    colcon build

Launch the Sensors:
Open a terminal on the QBot NVIDIA PC and run the following to start publishing sensor data:

    cd ~/ros2_ws/
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch qbot_sensors sensors.launch.py

2. Stream Data to Foxglove
Open a new terminal on the QBot NVIDIA PC and start the Foxglove WebSocket bridge:

    source /opt/ros/humble/setup.bash
    ros2 run foxglove_bridge foxglove_bridge

Connect from Windows PC:

    Open Foxglove Studio on your Windows PC.

    Open a new connection and enter ws://<ROBOT_IP_ADDRESS>:8765 (replace <ROBOT_IP_ADDRESS> with the IP used to SSH into the robot).

    View Camera Feed: Add an Image Panel and select the /camera/color/image_raw topic.

    View LiDAR Data: Add a 3D Panel. In the left sidebar settings, toggle the /scan topic to visible and ensure the Global Frame is set to base_link or lidar.

3. Control AMR Motions
Open a new terminal on the QBot NVIDIA PC to run the keyboard teleoperation script:

    cd /home/nvidia/tsukarm_dev
    python3 motion_with_keyboard.py
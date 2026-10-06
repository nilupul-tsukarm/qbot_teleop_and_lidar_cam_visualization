from Robotic_Arm.rm_robot_interface import *
import time

# Instantiate the RoboticArm class
arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
# Create the robotic arm connection and print the connection ID
handle = arm.rm_create_robot_arm("192.168.1.18", 8080)
print(handle.id)

while True:
    print(arm.rm_movej([-36.293, 52.014, -99.530, -30.951, 89.854, -50.833], 50, 0, 0, 0))
    time.sleep(1)
    print(arm.rm_movej([-2.287, 43.467, -77.835, -63.328, 83.172, 139.193], 50, 0, 0, 0))
    time.sleep(1)
    print(arm.rm_movej([-2.287, 43.467, -37.835, 0.0, 45.172, 139.193], 50, 0, 0, 0))
    time.sleep(1)
    print(arm.rm_movej([-2.287, 43.467, -77.835, -63.328, 83.172, 139.193], 50, 0, 0, 0))
    time.sleep(3)

arm.rm_delete_robot_arm()

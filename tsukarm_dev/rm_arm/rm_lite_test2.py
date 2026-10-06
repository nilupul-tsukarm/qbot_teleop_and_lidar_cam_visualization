from Robotic_Arm.rm_robot_interface import *

# Instantiate the RoboticArm class
arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
# Create the robotic arm connection and print the connection ID
handle = arm.rm_create_robot_arm("192.168.1.18", 8080)
print(handle.id)

print(arm.rm_movej_p([-3.101, 0, 2.472, 3.141, 0, 0], 20, 0, 0, 1))
print(arm.rm_movel([-3.101, 0, 2.472, 3.141, 0, 0], 20, 0, 0, 1))

arm.rm_delete_robot_arm()
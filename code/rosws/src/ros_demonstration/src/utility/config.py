#!/usr/bin/python

# Configuration file to tweak parameters

group_name = "Group_D"

# namespaces for different system units
lidar_driver_ns = '/lidar'
motor_driver_ns = '/motor'
imu_driver_ns = '/imu'
mag_driver_ns = '/mag'
movement_controller_ns = '/movement'

# --- movement ---

# once we are this distance in the vicinity of the desired position (x,y,z), we declare us arrived at the position (but not pose)
arrived_at_position_threshold = 0.005 # [m] 

# once the difference of our actual and the desired orientation is less than the threshold, we declare us arrived at the orientation
arrived_at_orientation_threshold = 5 # [degrees]

# if our heading to drive straight to the current heading is wrong by more than the threshold we will stop and only turn without linear motion
correct_heading_with_turn_only_threshold = 10 # [degrees]
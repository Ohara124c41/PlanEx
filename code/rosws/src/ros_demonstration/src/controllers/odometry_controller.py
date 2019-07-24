#!/usr/bin/python

import rospy
import math
import tf2_ros

import utility.pid as pid
import utility.config as cfg
from utility.geometry import convertMsgQuatToNumpyQuat, convertNumpyQuatToMsgQuat, quaternion_vector_mult
import utility.geometry as geometry


from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply

from ros_demonstration.srv import *

class OdometryController (object):
    """
    This class subscribes to all the odometry topics (e.g. wheel rpm, imu, camera, ...).
    The odometry topics will provide information about the current position of the robot.
    This class will interpret and fuse the sensor information and broadcast transforms via tf2 to inform other controllers (e.g. movement controller)
    """

    def __init__(self):
        # general values

        # set initial pose in odom frame (pose: [x,y,z], orientation:[x,y,z,w])
        self.pose = PoseStamped()
        self.pose.header.stamp = rospy.Time.now()
        self.pose.header.frame_id = 'odom'
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 0.0
        self.pose.pose.orientation.x = 0.0
        self.pose.pose.orientation.y = 0.0
        self.pose.pose.orientation.z = 0.0
        self.pose.pose.orientation.w = 1.0

        # initial velocity (=0)
        self.lastTwist = TwistStamped()
        self.lastTwist.header.stamp = rospy.Time.now()
        self.lastTwist.header.frame_id = 'base_link'
        self.lastTwist.twist.linear.x = 0
        self.lastTwist.twist.linear.y = 0
        self.lastTwist.twist.linear.z = 0
        self.lastTwist.twist.angular.x = 0
        self.lastTwist.twist.angular.y = 0
        self.lastTwist.twist.angular.z = 0

        # self.desired_pose = Pose([0,0,0],[0,0,0,1])
        # self.velocity_x = 0
        # self.velocity_y = 0
        # self.velocity_z = 0

        # # difference between world/odometry frame? and robot (x-axes)
        # self.angle_radians = 0
        # self.desired_angle_radians = 0
        # self.laser_distance = 0
        # self.desired_distance = 0
        # self.last_timestamp = 0

        # # subscribers
        # rospy.wait_for_message(group_name + 'imu_data', Imu)
        # rospy.wait_for_message(group_name + 'laser_scan', Float32)
        self.wheel_odom_sub = rospy.Subscriber(cfg.group_name + cfg.motor_driver_ns + '/wheel_odom', TwistStamped, self.wheelOdometryCallback)
        # self.imu_sub = rospy.Subscriber(group_name + 'imu_data', Imu, self.imu_callback)
        # self.laser_sub = rospy.Subscriber(group_name + 'laser_scan', Float32, self.laserCallback)
        # # services
        # rospy.wait_for_service(group_name + 'setPwm')
        # self.setMotorPwm = rospy.ServiceProxy(group_name + 'setPwm', motor_setPwm)
        # rospy.wait_for_service(group_name + 'turnToDegrees')
        # self.turnLidarToDegrees = rospy.ServiceProxy(group_name + 'turnToDegrees', lidar_turnToDegrees)
        # # set hardware to initial conditions
        # self.turnLidarToDegrees(0)
        # self.setMotorPwm(0,0)
        # # pid controllers
        # self.drivePid = pid.Pid(1, 0, 0.1, min_pwm, max_pwm)

    # def imu_callback(self, imu_msg):
    #         if(self.last_timestamp == 0):
    #             self.last_timestamp = imu_msg.header.stamp
    #             return

    #         delta_t = imu_msg.header.stamp - self.last_timestamp
    #         # self.velocity_x += imu_msg.linear_acceleration.x * delta_t
    #         # self.velocity_y += imu_msg.linear_acceleration.y * delta_t
    #         # self.velocity_z += imu_msg.linear_acceleration.z * delta_t

    #         # self.angle_radians += imu_msg.angular_velocity.z * delta_t

    #         # self.pose.position.x += math.cos(self.angle_radians) * self.velocity_x * delta_t
    #         # self.pose.position.y += math.sin(self.angle_radians) * self.velocity_y * delta_t
    #         # self.pose.position.z += self.velocity_z * delta_t

    # def laserCallback(self, laser_msg):
    #     self.laser_distance = laser_msg.data
    #     # rospy.loginfo("laser meas %lf", laser_msg.data)

    def updatePoseBasedOnWheelOdom(self, twistStamped):
        # see how much time has passed between current twist and last twist
        # change pose based on last twist and the time_delta
        time_delta = (twistStamped.header.stamp - self.lastTwist.header.stamp).to_sec()
        self.pose.header.stamp = twistStamped.header.stamp
        
        # calc how much we moved in base_link frame
        linear_movement_along_x = self.lastTwist.twist.linear.x * time_delta # [m/s * s = m], x-axis of base_link (robot)
        angular_movement_about_z = self.lastTwist.twist.angular.z * time_delta # [rad/s * s = rad], z-axis of base_link
        linear_movement_vec = [linear_movement_along_x, 0, 0]
        angular_movement_quat = quaternion_from_euler(0, 0, angular_movement_about_z)
        # rotate linear movement to convert it to odom frame and apply movement
        numpy_orientation = convertMsgQuatToNumpyQuat(self.pose.pose.orientation) # have to convert msg quat to numpy quat to be able to calculate
        rotated_vec = quaternion_vector_mult(numpy_orientation, linear_movement_vec)
        self.pose.pose.position.x += rotated_vec[0]
        self.pose.pose.position.y += rotated_vec[1]
        self.pose.pose.position.z += rotated_vec[2]
        

        # rotate orientation in odom pose by the amount we moved in the robot frame
        self.pose.pose.orientation = convertNumpyQuatToMsgQuat(quaternion_multiply(angular_movement_quat, numpy_orientation))

        # update last twist
        self.lastTwist = twistStamped

        # rospy.loginfo(self.pose.pose.position)
        # rospy.loginfo(euler_from_quaternion(convertMsgQuatToNumpyQuat(self.pose.pose.orientation)))

    def wheelOdometryCallback(self, twistStamped):
        # calculate new pose and publish to tf2
        self.updatePoseBasedOnWheelOdom(twistStamped)
        geometry.publishPoseToTf2(self.pose, 'base_link')
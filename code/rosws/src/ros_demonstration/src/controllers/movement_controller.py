#!/usr/bin/python

import rospy
import math

import utility.pid as pid
import utility.config as cfg
import utility.geometry as geometry

from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Quaternion

from ros_demonstration.srv import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply

import tf

class MovementController (object):
    """
    This class provides higher-level methods to move the robot.
    To execute these methods the low-level functions of the drivers are used.
    """

    def __init__(self):
        # general values
        linear_cmdvel_limit = 0.15 # [m/s]
        angular_cmdvel_limit = 20 # [degrees/s]

        # desired pose
        self.desired_poseStamped = PoseStamped()
        self.desired_poseStamped.header.stamp = rospy.Time(0)
        self.desired_poseStamped.header.frame_id = 'odom'
        self.desired_poseStamped.pose.position.x = 0 
        self.desired_poseStamped.pose.position.y = 0 
        self.desired_poseStamped.pose.position.z = 0 
        self.desired_poseStamped.pose.orientation.x = 0 
        self.desired_poseStamped.pose.orientation.y = 0 
        self.desired_poseStamped.pose.orientation.z = 0 
        self.desired_poseStamped.pose.orientation.w = 1

        # used services
        # rospy.wait_for_service(cfg.group_name + '/setPwm')
        self.setMotorPwm = rospy.ServiceProxy(cfg.group_name + cfg.motor_driver_ns + '/setPwm', motor_setPwm)
        # rospy.wait_for_service(cfg.group_name + '/turnToDegrees')
        self.turnLidarToDegrees = rospy.ServiceProxy(cfg.group_name + cfg.lidar_driver_ns + '/turnToDegrees', lidar_turnToDegrees)

        # advertised services
        self.setPwmService = rospy.Service(cfg.group_name + cfg.movement_controller_ns + '/drive', movement_drive, self.drive_viaService)
        self.setPwmService = rospy.Service(cfg.group_name + cfg.movement_controller_ns + '/brake', movement_brake, self.brake_viaService)
        self.setPwmService = rospy.Service(cfg.group_name + cfg.movement_controller_ns + '/turnToDegrees', movement_turnToDegrees, self.turnToDegrees_viaService)
        self.setPwmService = rospy.Service(cfg.group_name + cfg.movement_controller_ns + '/turnByDegrees', movement_turnByDegrees, self.turnByDegrees_viaService)
        self.setPwmService = rospy.Service(cfg.group_name + cfg.movement_controller_ns + '/driveToPose', movement_driveToPose, self.driveToPose_viaService)

        # set hardware to initial conditions
        self.turnLidarToDegrees(0)
        self.setMotorPwm(0,0)
        # pid controllers
        self.velocity_pid = pid.Pid(1, 0, 0.1, -linear_cmdvel_limit, linear_cmdvel_limit)
        self.steering_pid = pid.Pid(1, 0, 0.1, -angular_cmdvel_limit, angular_cmdvel_limit)
        
        # timer to update odometry
        rospy.Timer(rospy.Duration(0.1), self.publish_cmdvel)
        rospy.Timer(rospy.Duration(0.1), self.publish_desired_pose_to_tf2)
        self.listener = tf.TransformListener()

        self.cmdVelPublisher = rospy.Publisher(cfg.group_name + cfg.motor_driver_ns + '/cmd_vel', TwistStamped, queue_size=10)
        # self.test_val = 0.0

    def publish_cmdvel(self, event):
        """Looks up the difference between the desired pose and the actual pose (both in the odom frame) and takes movement to match both poses""" 
        try:
            (trans,rot) = self.listener.lookupTransform('/base_link', '/desired_pose', rospy.Time(0))
            twistStamped = self.make_movement_decision(trans, rot)
            self.cmdVelPublisher.publish(twistStamped)
            #rospy.loginfo(twistStamped)

        except Exception as e:#(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn(e.message)
            pass

    def publish_desired_pose_to_tf2(self, event):
        # publish new desired pose to tf2
        self.desired_poseStamped.header.stamp = rospy.get_rostime()# + rospy.Duration(self.test_val)
        # self.test_val += 0.1
        # rospy.loginfo("x: %lf", self.desired_poseStamped.pose.position.x)
        geometry.publishPoseToTf2(self.desired_poseStamped, 'desired_pose')

    def make_movement_decision(self, translation, rotation):
        """ 
        Uses the transform between actual and desired pose to decide how to move.
        @param translation: Carries the position (x,y,z) of the desired_position expressed in the base_link frame
        @param rotation: Carries the orientation (quaternion) of the desired_position expressed in the base_link frame
        """

        twistStamped = TwistStamped()
        twistStamped.header.stamp = rospy.get_rostime()
        twistStamped.header.frame_id = 'base_link'

        # before we assume the pose orientation, we want to drive straight to the goal
        # difference between the robots orientation and heading straight to the goal pose position (not the difference to the goal pose orientation)
        front_to_goal_position_angular_difference = math.atan2(translation[1], translation[0])
        # distance between us and the goal (straight line)
        euclidean_distance = math.sqrt(translation[0] ** 2 + translation[1] ** 2)
        
        # rospy.loginfo(translation)
        # rospy.loginfo(rotation)
        # rospy.loginfo(front_to_goal_position_angular_difference)
        # rospy.loginfo(euclidean_distance)

        if(euclidean_distance > cfg.arrived_at_position_threshold):
            # we didn't arrive at the position yet
            if(abs(front_to_goal_position_angular_difference) > math.radians(cfg.correct_heading_with_turn_only_threshold)):
                # if our heading is wrong by more than a threshold we will turn on the spot before driving
                # note that this is not a blocking call. once we are within the threshold, the condition changes and this line will not be executed anymore.
                twistStamped.twist.linear.x = 0
                twistStamped.twist.angular.z = self.steering_pid.calculate(0, front_to_goal_position_angular_difference)
            else:
                # we are headed towards the goal, drive forward and steer to stay on track
                twistStamped.twist.linear.x = self.velocity_pid.calculate(0, euclidean_distance)
                twistStamped.twist.angular.z = self.steering_pid.calculate(0, front_to_goal_position_angular_difference)
        else:
            # we are at the position, align with the desired orientation
            # calculate difference between robot orientation and goal pose orientation
            # current_orientation_inv = Quaternion()
            # current_orientation_inv.x = self.poseStamped.pose.orientation.x
            # current_orientation_inv.y = self.poseStamped.pose.orientation.y
            # current_orientation_inv.z = self.poseStamped.pose.orientation.z
            # current_orientation_inv.w = -self.poseStamped.pose.orientation.z

            # cur_orientation_inv_np = geometry.convertMsgQuatToNumpyQuat(current_orientation_inv)
            # desired_orientation_np = geometry.convertMsgQuatToNumpyQuat(self.desired_poseStamped.pose.orientation)

            # front_to_goal_orientation_quaternion = quaternion_multiply(desired_orientation_np, cur_orientation_inv_np)
            front_to_goal_orientation_angular_difference = euler_from_quaternion(rotation)[2] # get z-rotation (yaw)

            twistStamped.twist.linear.x = 0
            twistStamped.twist.angular.z = self.steering_pid.calculate(0, front_to_goal_orientation_angular_difference)

        return twistStamped

    def drive_viaService(self, request):
        """ Service function for drive() """

        return self.drive(request.distance)

    def drive(self, distance):
        """ 
        Moves the robot a certain distance [m] in a straight direction.
        What actually happens: The desired pose is moved by the distance.
        """
        dist_vec = [distance, 0, 0] # x-axis points to the front
        # rotate distance to convert it to odom frame and apply movement
        numpy_orientation = geometry.convertMsgQuatToNumpyQuat(self.desired_poseStamped.pose.orientation) # have to convert msg quat to numpy quat to be able to calculate
        rotated_vec = geometry.quaternion_vector_mult(numpy_orientation, dist_vec)
        # update desired pose
        self.desired_poseStamped.pose.position.x += rotated_vec[0]
        self.desired_poseStamped.pose.position.y += rotated_vec[1]
        self.desired_poseStamped.pose.position.z += rotated_vec[2]

        return True

    def brake_viaService(self, request):
        """ Service function for brake """

        return self.brake()

    def brake(self):
        """
        Makes the robot stop immediately.
        What actually happens: The desired pose is set to the current pose.
        """
        rospy.loginfo("braking")

        try:
            (trans,rot) = self.listener.lookupTransform('/odom', '/base_link', rospy.Time(0))

            self.desired_poseStamped.pose.position.x = trans[0]
            self.desired_poseStamped.pose.position.y = trans[1]
            self.desired_poseStamped.pose.position.z = trans[2]
            self.desired_poseStamped.pose.orientation.x = rot[0]
            self.desired_poseStamped.pose.orientation.y = rot[1]
            self.desired_poseStamped.pose.orientation.z = rot[2]
            self.desired_poseStamped.pose.orientation.w = rot[3]
            return True

        except Exception as e:#(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Couldn't brake!")
            rospy.logwarn(e.message)
            return False

    def turnToDegrees_viaService(self, request):
        """ Service function for turnToDegrees() """

        return self.turnToDegrees(request.degrees)

    def turnToDegrees(self, degrees):
        """
        Makes the robot turn on the spot to the given degrees (relative to the initial frame).
        Positive values turn counter-clockwise, 
        negative values turn clockwise (mathematical sense of rotation)
        """

        radians = math.radians(degrees)
        return self.turnToRadians(radians)

    def turnToRadians(self, radians):
        """
        Makes the robot turn on the spot to the given radians (relative to the initial frame)
        Positive values turn counter-clockwise, 
        negative values turn clockwise (mathematical sense of rotation)
        """

        rospy.loginfo("turning to %lf degrees", math.degrees(radians))

        rotation_quaternion = quaternion_from_euler(0, 0, radians)
        self.desired_poseStamped.pose.orientation = geometry.convertNumpyQuatToMsgQuat(rotation_quaternion)

        return True

    def turnByDegrees_viaService(self, request):
        """ Service function for turnByDegrees() """

        return self.turnByDegrees(request.degrees)

    def turnByDegrees(self, degrees):
        """
        Makes the robot turn on the spot by the given degrees (relative to the initial frame).
        Positive values turn counter-clockwise, 
        negative values turn clockwise (mathematical sense of rotation)
        """

        radians = math.radians(degrees)
        return self.turnByRadians(radians)

    def turnByRadians(self, radians):
        """
        Makes the robot turn on the spot by the given radians (relative to the initial frame)
        Positive values turn counter-clockwise, 
        negative values turn clockwise (mathematical sense of rotation)
        """

        rospy.loginfo("turning by %lf degrees", math.degrees(radians))

        rotation_quaternion = quaternion_from_euler(0, 0, radians)
        desired_orientation_np = geometry.convertMsgQuatToNumpyQuat(self.desired_poseStamped.pose.orientation)
        new_desired_orientation_np = tf.transformations.quaternion_multiply(rotation_quaternion, desired_orientation_np)
        self.desired_poseStamped.pose.orientation = geometry.convertNumpyQuatToMsgQuat(new_desired_orientation_np)

        return True

    def driveToPose_viaService(self, request):
        """ Service function for driveToPose() """

        pose = Pose()
        pose.position.x = request.x
        pose.position.y = request.y
        pose.position.z = 0
        pose.orientation = geometry.convertNumpyQuatToMsgQuat(quaternion_from_euler(0, 0, math.radians(request.theta_deg)))

        return self.driveToPose(pose)

    def driveToPose(self, pose):
        """ Drives the robot to a desired pose in the odom frame """
        self.desired_poseStamped.pose = pose

        return True

    # Other Ideas:
    # - drive a curve (smart parameters?, necessary?) 
    # - follow trajectory (too high level?)
#!/usr/bin/python
import rospy

import drivers.imu_driver as imu_driver
import drivers.motor_driver as motor_driver

import controllers.movement_controller as mc
import controllers.odometry_controller as oc

class Robot ( object ):
    """High-level class that fuses all functionality of the robot"""

    def __init__( self ):
        rospy.init_node("robot")
        rospy.on_shutdown(self.destruct)

        # controllers
        self.movement_controller = mc.MovementController()
        self.odometry_conrtoller = oc.OdometryController()

    def destruct(self):
        rospy.loginfo("Destructing robot")

    def execute_plan(self, plan):
        """Takes actions based on the given plan"""
        
        if(plan == "assignment1"):
            rospy.loginfo("executing assignment 1")
            distance = 1000 #[mm]
            self.movement_controller.drive(distance)
            rospy.sleep(4.) #[s]
            self.movement_controller.brake()

        if(plan == "test"):
            rospy.loginfo("doing test")

        rospy.spin()

if __name__ == '__main__':
    # just testing if it works
    robot = Robot()
#!/usr/bin/python
import rospy
import math
import time

if __name__ == '__main__' and __package__ is None:
    from os import sys, path
    sys.path.append(path.abspath(path.join(path.dirname(__file__), '..')))

from utility import i2c as i2c
import utility.register_data as rd
import utility.config as cfg

from std_msgs.msg import Float32
from ros_demonstration.srv import *

class LidarDriver(object):
    """
    This class provides low-level control and information of the motors.
    """

    # NOTE: it will make sense to use the laserScan msg from ros for the lidar
    # NOTE: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html

    def __init__ (self):
        rospy.init_node("lidar_driver")
        rospy.loginfo("Starting LIDAR driver")
        rospy.on_shutdown(self.destruct)

        self.rate = rospy.Rate(10)
        self.min_degrees = -80 # right (datasheet says -90 but setting that position doesnt yield any movement)
        self.max_degrees =  80 # left
        self.speed = 425 # [degrees/s]
        self.laserScan = Float32()

        self.publisher = rospy.Publisher(cfg.group_name + cfg.lidar_driver_ns + '/laser_scan', Float32, queue_size=10)
        turnToDegreesService = rospy.Service(cfg.group_name + cfg.lidar_driver_ns + '/turnToDegrees', lidar_turnToDegrees, self.turnToDegrees_viaService)
        turnToRadiansService = rospy.Service(cfg.group_name + cfg.lidar_driver_ns + '/turnToRadians', lidar_turnToRadians, self.turnToRadians_viaService)
        turnByDegreesService = rospy.Service(cfg.group_name + cfg.lidar_driver_ns + '/turnByDegrees', lidar_turnByDegrees, self.turnByDegrees_viaService)
        turnByRadiansService = rospy.Service(cfg.group_name + cfg.lidar_driver_ns + '/turnByRadians', lidar_turnByRadians, self.turnByRadians_viaService)
        getPositionDegreesService = rospy.Service(cfg.group_name + cfg.lidar_driver_ns + '/getPositionDegrees', lidar_getPositionDegrees, self.getPositionDegrees_viaService)
        getPositionRadiansService = rospy.Service(cfg.group_name + cfg.lidar_driver_ns + '/getPositionRadians', lidar_getPositionRadians, self.getPositionRadians_viaService)
        getDistanceFrontService = rospy.Service(cfg.group_name + cfg.lidar_driver_ns + '/getDistanceFront', lidar_getDistanceFront, self.getDistanceFront_viaService)
        getDistanceBackService = rospy.Service(cfg.group_name + cfg.lidar_driver_ns + '/getDistanceBack', lidar_getDistanceBack, self.getDistanceBack_viaService)
        
        self.run()

    def destruct(self):
        self.turnToDegrees(0)
        rospy.loginfo("Destructing LIDAR driver")

    def run(self):
        self.turnToDegrees(0)
        while(not rospy.is_shutdown()):
            # publish the new laser scan
            self.laserScan.data = self.getDistanceFront();
            self.publisher.publish(self.laserScan)
            self.rate.sleep()

    def turnToDegrees_viaService(self, request):
        """Service function for turnToDegrees"""

        return self.turnToDegrees(request.degrees)

    def turnToDegrees(self, degrees):
        """
        Makes the Lidar move to an absolute position.
        0 degrees makes the Lidar face forward, positive values turn it left, negative right (mathematical rotation)
        Only integer values are allowed.
        The motor speed is ~425 degrees/s. For the 170 degrees turn it needs ~400ms.
        Max value:  80 degrees
        Min value: -90 degrees
        """
        degrees = int(degrees)

        if(self.min_degrees <= degrees <= self.max_degrees):
            i2c.write8(rd.LIDAR_I2C_ADDRESS, rd.LIDAR_SENSOR_POSITION, degrees)
            return True
        else:
            return False

    def turnToRadians_viaService(self, request):
            """Service function for turnToRadians"""

            return self.turnToRadians(request.radians)

    def turnToRadians(self, radians):
        """
        Makes the Lidar move to an absolute position.
        0 degrees makes the Lidar face forward, positive values turn it left, negative right (mathematical rotation)
        Only integer degree values are allowed -> loses precision.
        The motor speed is ~425 degrees/s. For the 170 degrees turn it needs ~400ms.
        Max value:  1.39626 (=  80 degrees)
        Min value: -1.57079 (= -90 degrees)
        """

        degrees = int(math.degrees(radians))
        return self.turnToDegrees(degrees)

    def turnByDegrees_viaService(self, request):
        """Service function for turnByDegrees"""

        return self.turnByDegrees(request.degrees)

    def turnByDegrees(self, degrees):
        """
        Makes the Lidar move relative to its current position.
        Negative values turn it right, positive left.
        Only integer values are allowed.
        If it would move out of range, it will move to the max range in that direction.
        The motor speed is ~425 degrees/s. For the 170 degrees turn it needs ~400ms.
        """

        degrees = int(degrees)
        
        # TODO: it might be smart to save the current position locally and not read via i2c (faster)
        current_position = self.getPositionDegrees()
        min_degrees = self.min_degrees
        max_degrees = self.max_degrees
        new_position = current_position + degrees

        if ( new_position < min_degrees ): # hit minimum, drive to minimum
            self.turnToDegrees(min_degrees)
            return False
        elif ( new_position > max_degrees ): # hit maximum, drive to maximum
            self.turnToDegrees(max_degrees)
            return False
        else: # in range
            self.turnToDegrees(new_position)
            return True

    def turnByRadians_viaService(self, request):
            """Service function for turnByRadians"""

            return self.turnByRadians(request.radians)

    def turnByRadians(self, radians):
        """
        Makes the Lidar move relative to its current position.
        Negative values turn it right, positive left.
        Only integer degree values are allowed -> loses precision.
        If it would move out of range, it will move to the max range in that direction.
        The motor speed is ~425 degrees/s. For the 170 degrees turn it needs ~400ms.
        """

        degrees = int(math.degrees(radians))
        return self.turnByDegrees(degrees)

    def getPositionDegrees_viaService(self, request):
        """Service function for getPositionDegrees()"""

        return self.getPositionDegrees()

    def getPositionDegrees(self):
        """Gets the current sensor position in degrees"""

        degrees = i2c.read_int8(rd.LIDAR_I2C_ADDRESS, rd.LIDAR_SENSOR_POSITION)
        return degrees

    def getPositionRadians_viaService(self, request):
        """Service function for getPositionRadians"""

        return self.getPositionRadians()

    def getPositionRadians(self):
        """Gets the current sensor position in radians"""

        degrees = self.getPositionDegrees()
        radians = math.radians(degrees)
        return radians

    def getDistanceFront_viaService(self, request):
        """Service function for getDistnaceFront"""

        return self.getDistanceFront()

    def getDistanceFront(self):
        """
        Returns the last distance the front sensor has measured.
        Values over 4m are unreliable and will be discarded (return None).
        The unit is mm. The sensor works at ~10 Hz.
        """

        distance = i2c.read_uint16(rd.LIDAR_I2C_ADDRESS, rd.LIDAR_FRONT_DISTANCE, False)
        
        if (distance > 4000):
            return float("nan")
        
        return distance

    def getDistanceBack_viaService(self, request):
        """Service function for getDistnaceBack"""

        return self.getDistanceBack()

    def getDistanceBack(self):
        """
        Returns the last distance the back sensor has measured.
        Values over 4m are unreliable and will be discarded (return None).
        The unit is mm. The sensor works at ~10 Hz.
        """
        
        # NOTE: back sensor might not exist!

        distance = i2c.read_uint16(rd.LIDAR_I2C_ADDRESS, rd.LIDAR_BACK_DISTANCE, False)
        
        if (distance > 4000):
            return float("nan")
        
        return distance

    def scan(self):
        """Makes the lidar sweep the area continuously to be able to publish a laserScan"""
        # TODO
        pass

if __name__ == '__main__':
    lidar_driver = LidarDriver()
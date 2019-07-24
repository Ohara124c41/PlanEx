#!/usr/bin/python
import rospy
import math 
from time import sleep
from ros_demonstration.srv import *
from geometry_msgs.msg import TwistStamped

if __name__ == '__main__' and __package__ is None:
    from os import sys, path
    # add parent folder to system path to be able to import from other subfolders
    sys.path.append(path.abspath(path.join(path.dirname(__file__), '..')))

import utility.i2c as i2c
import utility.register_data as rd
import utility.config as cfg

class MotorDriver(object):
    """
    This class provides low-level control and information of the motors.
    """

    def __init__ (self):
        rospy.init_node('motor_driver')
        rospy.loginfo("Starting Motor driver")
        rospy.on_shutdown(self.destruct)
        self.wheel_radius = 0.015 # [m] #TODO next 4 values are just guessed and not tested
        self.track = 0.03 # [m] (Spurweite, Distanz zwischen den Raedern)
        self.max_rpm = 100
        self.min_rpm = -100
        self.rate = rospy.Rate(10) # publishing rate
        self.setPwmService = rospy.Service(cfg.group_name + cfg.motor_driver_ns + '/setPwm', motor_setPwm, self.setPwm_viaService)
        self.setRpmService = rospy.Service(cfg.group_name + cfg.motor_driver_ns + '/setRpm', motor_setRpm, self.setRpm_viaService)
        self.setPidService = rospy.Service(cfg.group_name + cfg.motor_driver_ns + '/setPid', motor_setPid, self.setPid_viaService)
        self.getPwmService = rospy.Service(cfg.group_name + cfg.motor_driver_ns + '/getPwm', motor_getPwm, self.getPwm_viaService)
        self.getRpmService = rospy.Service(cfg.group_name + cfg.motor_driver_ns + '/getRpm', motor_getRpm, self.getRpm_viaService)
        self.getPidService = rospy.Service(cfg.group_name + cfg.motor_driver_ns + '/getPid', motor_getPid, self.getPid_viaService)
        self.twistSubscriber = rospy.Subscriber(cfg.group_name + cfg.motor_driver_ns + '/cmd_vel', TwistStamped, self.cmdVelCallback)
        self.twistPublisher = rospy.Publisher(cfg.group_name + cfg.motor_driver_ns + '/wheel_odom', TwistStamped, queue_size=10)        

        self.run()

    def run(self):
        while(not rospy.is_shutdown()):
            # publish the new twist msg            
            twistStamped = self.getCurrentTwist();
            self.twistPublisher.publish(twistStamped)
            self.rate.sleep()

    def destruct(self):
        # make sure motors are turned off
        rospy.loginfo("Destructing Motor driver (turning off motors)")
        self.setPwm(0, 0)

    def setPwm_viaService(self, request):
        """ Service function for setPwm() """
        return self.setPwm(request.leftPwm, request.rightPwm)

    def setPwm(self, left_pwm, right_pwm):
        """
        Sets the pwm signals to the motors.
        Deactivates the PID Controller.
        To set only one wheel, set None for the other.
        Max value: 1000 (forward)
        Min value: -1000 (backward)
        """

        left_in_range = -1000 <= left_pwm <= 1000
        right_in_range = -1000 <= right_pwm <= 1000

        if(not left_in_range and not right_in_range):
            return False

        if(left_in_range):
            i2c.write16(rd.MOTOR_I2C_ADDRESS, rd.MOTOR_LEFT_PWM, left_pwm, False)

        if(right_in_range):
            i2c.write16(rd.MOTOR_I2C_ADDRESS, rd.MOTOR_RIGHT_PWM, right_pwm, False)

        return True

    def getPwm_viaService(self, request):
        """Service function for getPwm()"""

        return self.getPwm()

    def getPwm(self):
        """Gets the current PWM signals send to the motors"""

        left_pwm = i2c.read_int16(rd.MOTOR_I2C_ADDRESS, rd.MOTOR_LEFT_PWM, False)
        right_pwm = i2c.read_int16(rd.MOTOR_I2C_ADDRESS, rd.MOTOR_RIGHT_PWM, False)
        return left_pwm, right_pwm

    def setRpm_viaService(self, request):
        """Service function for setRpm()"""

        return self.setRpm(request.leftRpm, request.rightRpm)

    def setRpm(self, left_rpm, right_rpm):
        """
        Sets the rpm of the motors.
        The unit is 100/min (e.g. a value of 123 stands for 1.23 rpm)  FIXME: this is not up to date anymore (1/min now), maybe min/max value changed, too?
        Activates the PID controller.
        To set only one wheel, set None for the other.
        Max value: 32767 (forward)
        Min value: -32768 (backward)
        """

        left_rpm = int(left_rpm)
        right_rpm = int(right_rpm)

        left_in_range = -32768 <= left_rpm <= 32767 
        right_in_range = -32768 <= right_rpm <= 32767

        if(not left_in_range and not right_in_range):
            return False

        if(left_in_range):
            i2c.write16(rd.MOTOR_I2C_ADDRESS, rd.MOTOR_LEFT_RPM, left_rpm, False)

        if(right_in_range):
            i2c.write16(rd.MOTOR_I2C_ADDRESS, rd.MOTOR_RIGHT_RPM, right_rpm, False)

        return True

    def getRpm_viaService(self, request):
        """Service function for getRpm()"""

        return self.getRpm()

    def getRpm(self):
        """Gets the current rpm of the motors"""

        left_rpm = i2c.read_int16(rd.MOTOR_I2C_ADDRESS, rd.MOTOR_LEFT_RPM, False)
        right_rpm = i2c.read_int16(rd.MOTOR_I2C_ADDRESS, rd.MOTOR_RIGHT_RPM, False)
        return left_rpm, right_rpm

    def setPid_viaService(self, request):
        """Service function for setPid()"""
        return self.setPid(request.side, request.P, request.I, request.D)

    def setPid(self, side, p, i, d):
        """
        Sets the PID controller values.
        The unit is 1000/1 (e.g. a value of 1234 stands for a value of 1.234)
        Set to None if you want to ignore one parameter.
        Side can be 'left' or 'right' and defines the wheel to change.
        Max PID value: 32767
        Min PID value: -32768
        """

        min = -32768
        max = 32767

        if(side == 'left'):
            if(min <= p <= max):
                i2c.write16(rd.MOTOR_I2C_ADDRESS, rd.MOTOR_LEFT_P_VALUE, p, False)
            if(min <= i <= max):
                i2c.write16(rd.MOTOR_I2C_ADDRESS, rd.MOTOR_LEFT_I_VALUE, i, False)
            if(min <= d <= max):
                i2c.write16(rd.MOTOR_I2C_ADDRESS, rd.MOTOR_LEFT_D_VALUE, d, False)
            return True

        elif(side == 'right'):
            if(min <= p <= max):
                i2c.write16(rd.MOTOR_I2C_ADDRESS, rd.MOTOR_RIGHT_P_VALUE, p, False)
            if(min <= i <= max):
                i2c.write16(rd.MOTOR_I2C_ADDRESS, rd.MOTOR_RIGHT_I_VALUE, i, False)
            if(min <= d <= max):
                i2c.write16(rd.MOTOR_I2C_ADDRESS, rd.MOTOR_RIGHT_D_VALUE, d, False)
            return True

        else:
            return False

    def getPid_viaService(self, request):
        """Service function for getPID()"""
        return self.getPid(request.side)

    def getPid(self, side):
        """Returns the current PID values for the controller of the specified wheel side"""

        p = None
        i = None
        d = None

        if(side == 'left'):
            p = i2c.read_int16(rd.MOTOR_I2C_ADDRESS, rd.MOTOR_LEFT_P_VALUE, False)
            i = i2c.read_int16(rd.MOTOR_I2C_ADDRESS, rd.MOTOR_LEFT_I_VALUE, False)
            d = i2c.read_int16(rd.MOTOR_I2C_ADDRESS, rd.MOTOR_LEFT_D_VALUE, False)
        
        elif(side == 'right'):
            p = i2c.read_int16(rd.MOTOR_I2C_ADDRESS, rd.MOTOR_RIGHT_P_VALUE, False)
            i = i2c.read_int16(rd.MOTOR_I2C_ADDRESS, rd.MOTOR_RIGHT_I_VALUE, False)
            d = i2c.read_int16(rd.MOTOR_I2C_ADDRESS, rd.MOTOR_RIGHT_D_VALUE, False)

        else:
            p = -1
            i = -1
            d = -1

        return p, i, d

    def actuateWheelsBasedOnTwistMsg(self, twistMsg):
        # calculate rpm values
        v_robot = twistMsg.twist.linear.x
        w_robot = twistMsg.twist.angular.z
        rpm_wheel_linear = v_robot / (2 * math.pi * self.wheel_radius)
        rpm_wheel_angular = w_robot * (self.track/2) / self.wheel_radius 
        rpm_left = 60*(rpm_wheel_linear - rpm_wheel_angular)
        rpm_right = 60*(rpm_wheel_linear + rpm_wheel_angular)
        # limit to min/max values
        # TODO if angular part leads to excess of max/min vel, we could compensate that via the other wheel (increase/decrease speed)
        rpm_left_limited = min(max(rpm_left, self.min_rpm), self.max_rpm)
        rpm_right_limited = min(max(rpm_right, self.min_rpm), self.max_rpm)
        # actuate motors
        self.setRpm(rpm_left_limited, rpm_right_limited)
        rospy.loginfo("rpm left %lf, rpm right %lf", rpm_left_limited, rpm_right_limited)

    def getCurrentTwist(self):
        # get the current rpm of the wheels
        left_rpm, right_rpm = self.getRpm()
        left_rps = left_rpm / 60.0
        right_rps = right_rpm / 60.0
        # rospy.loginfo("l rps %lf, r rps %lf", left_rps, right_rps)
        avg_rps = (left_rps + right_rps) / 2 # for linear term
        rps_diff = (right_rps - left_rps) / 2 # for angular term (divide by 2 because both velocities are on a circle and don't add up)

        v_robot = avg_rps * 2 * math.pi * self.wheel_radius
        w_robot = rps_diff * self.wheel_radius / (self.track/2)
        # convert to twist
        twistMsg = TwistStamped()
        twistMsg.header.stamp = rospy.Time.now()
        twistMsg.header.frame_id = 'base_link'
        twistMsg.twist.linear.x = v_robot # [m/s]
        twistMsg.twist.linear.y = 0
        twistMsg.twist.linear.z = 0
        twistMsg.twist.angular.x = 0
        twistMsg.twist.angular.y = 0
        twistMsg.twist.angular.z = w_robot # [rad/s]

        return twistMsg

    def cmdVelCallback(self, cmdVelMsg):
        self.actuateWheelsBasedOnTwistMsg(cmdVelMsg)

if __name__ == '__main__':
    motor_driver = MotorDriver()
#!/usr/bin/python
import rospy

if __name__ == '__main__' and __package__ is None:
    from os import sys, path
    sys.path.append(path.abspath(path.join(path.dirname(__file__), '..')))

import utility.i2c as i2c
import utility.register_data as rd
import utility.register_commands as rc
import utility.config as cfg

from time import sleep


from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose

class ImuDriver (object):
    """
    This class provides low-level control and information of the IMU (inertial measurement unit).
    """

    def __init__ (self):
        rospy.init_node("imu_driver")
        rospy.loginfo("Starting IMU Driver")
        rospy.on_shutdown(self.destruct)
        self.imu_publisher = rospy.Publisher(cfg.group_name + cfg.imu_driver_ns + '/imu_data', Imu, queue_size=10)
        self.mag_publisher = rospy.Publisher(cfg.group_name + cfg.imu_driver_ns + '/mag_data', Vector3, queue_size=10)
#        self.publisher = rospy.Publisher(cfg.group_name + '/imu_pose', Pose, queue_size=10)
        self.rate = rospy.Rate(10)
        self.current_imu_msg = Imu()
        self.current_imu_msg.header.frame_id = cfg.group_name + '/imu_frame'
        self.current_imu_msg.orientation.x = 0.0
        self.current_imu_msg.orientation.y = 0.0
        self.current_imu_msg.orientation.z = 0.0
        self.current_imu_msg.orientation.w = 1.0

        # self.current_mag_msg = MagneticField()
        # self.current_mag_msg.header.frame_id = cfg.group_Name + '/mag_frame'
        # self.current_mag_msg.header.stamp = rospy.Time.now()
        # self.current_mag_msg.
        self.current_mag_msg = Vector3()
        self.current_mag_msg.x = 0
        self.current_mag_msg.y = 0
        self.current_mag_msg.z = 0

        self.activate()
        self.run()

    def destruct(self):
        rospy.loginfo("Destructing IMU driver")
        self.deactivate()

    def activate(self):
        # reset imu
        i2c.write8(rd.IMU_I2C_ADDRESS, rd.IMU_POWERMANAGEMENT1, rc.IMU_RESET)
        # activate accelerometers and gyros
        i2c.write8(rd.IMU_I2C_ADDRESS, rd.IMU_POWERMANAGEMENT2, rc.IMU_ACTIVATE)
        # deactivate internal imu i2c master interface and enable magnetometer i2c bypass multiplexer
        i2c.write8(rd.IMU_I2C_ADDRESS, rd.IMU_I2C_MASTER_INTERFACE, rc.IMU_DISABLE_INTERNAL_I2C_MASTER)
        i2c.write8(rd.IMU_I2C_ADDRESS, rd.IMU_I2C_BYPASS, rc.IMU_ACTIVATE_MAG_I2C_BYPASS)
        # init single measurement of magnetometer
        i2c.write8(rd.IMU_MAG_I2C_ADDRESS, rd.IMU_MAG_CNTL1, rc.IMU_MAG_POWERDOWN)
        i2c.write8(rd.IMU_MAG_I2C_ADDRESS, rd.IMU_MAG_CNTL1, rc.IMU_MAG_ACTIVATE_16BIT_SINGLE)
        # rospy.logwarn(i2c.read_uint8(rd.IMU_MAG_I2C_ADDRESS, rd.IMU_MAG_CNTL1))

    def deactivate(self):
        # deactivate accelerometers and gyros
        i2c.write8(rd.IMU_I2C_ADDRESS, rd.IMU_POWERMANAGEMENT2, rc.IMU_DEACTIVATE)
        # deactivate magnetometer
        i2c.write8(rd.IMU_I2C_ADDRESS, rd.IMU_MAG_CNTL1, rc.IMU_MAG_POWERDOWN)

    def run (self):
        while(not rospy.is_shutdown()):
            # get imu values and set them in the current imu msg
            gyroscope_x, gyroscope_y, gyroscope_z = self.getAngularVelocity()
            self.current_imu_msg.angular_velocity.x = gyroscope_x
            self.current_imu_msg.angular_velocity.y = gyroscope_y
            self.current_imu_msg.angular_velocity.z = gyroscope_z

            acceleration_x, acceleration_y, acceleration_z = self.getLinearAcceleration()
            self.current_imu_msg.linear_acceleration.x = acceleration_x
            self.current_imu_msg.linear_acceleration.y = acceleration_y
            self.current_imu_msg.linear_acceleration.z = acceleration_z

            # publish the new imu msg
            self.imu_publisher.publish(self.current_imu_msg)

            # get magnetometer values
            
            drdy = self.getMagnetometerDataReadyBit()
            if(drdy):
                # mag data is ready, get mag data
                mag_x, mag_y, mag_z = self.getMagneticFieldVector()
                self.current_mag_msg.x = mag_x
                self.current_mag_msg.y = mag_y
                self.current_mag_msg.z = mag_z
                self.mag_publisher.publish(self.current_mag_msg)
                # request another single measurement
                i2c.write8(rd.IMU_MAG_I2C_ADDRESS, rd.IMU_MAG_CNTL1, rc.IMU_MAG_ACTIVATE_16BIT_SINGLE)
            
            # rospy.logwarn("data ready %d", self.getMagnetometerDataReadyBit())
            # rospy.logwarn("overflow %d", self.getMagnetometerOverflowBit())
            # publish the new mag msg
            self.rate.sleep()

    def getAngularVelocity(self):
        """Returns the processed angular acceleration from the sensor"""

        # TODO: check if the return values are actually the angular velocity

        # get raw data
        gyroscope_x_raw = i2c.read_int16(rd.IMU_I2C_ADDRESS, rd.IMU_GYRO_X, True)
        gyroscope_y_raw = i2c.read_int16(rd.IMU_I2C_ADDRESS, rd.IMU_GYRO_Y, True)
        gyroscope_z_raw = i2c.read_int16(rd.IMU_I2C_ADDRESS, rd.IMU_GYRO_Z, True)
        # scale to get actual value
        gyroscope_x = gyroscope_x_raw / 131.0
        gyroscope_y = gyroscope_y_raw / 131.0
        gyroscope_z = gyroscope_z_raw / 131.0

        return gyroscope_x, gyroscope_y, gyroscope_z

    def getLinearAcceleration(self):
        """Returns the processed linear acceleration from the sensor"""

        # TODO: check if the return values are actually the angular velocity

        # get raw data
        acceleration_x_raw = i2c.read_int16(rd.IMU_I2C_ADDRESS, rd.IMU_ACCELERATION_X, True)
        acceleration_y_raw = i2c.read_int16(rd.IMU_I2C_ADDRESS, rd.IMU_ACCELERATION_Y, True)
        acceleration_z_raw = i2c.read_int16(rd.IMU_I2C_ADDRESS, rd.IMU_ACCELERATION_Z, True)
        # scale to get actual value
        acceleration_x = acceleration_x_raw / 16384.0
        acceleration_y = acceleration_y_raw / 16384.0
        acceleration_z = acceleration_z_raw / 16384.0

        return acceleration_x, acceleration_y, acceleration_z

    # TODO: whats up with this, maybe this is the angular velocity?: -> uses gravity accel to see how we're oriented relative to that
    #rospy.loginfo "X Rotation: " , get_x_rotation(beschleunigung_xout_skaliert, beschleunigung_yout_skaliert, beschleunigung_zout_skaliert)
    #rospy.loginfo "Y Rotation: " , get_y_rotation(beschleunigung_xout_skaliert, beschleunigung_yout_skaliert, beschleunigung_zout_skaliert)

    def getMagneticFieldVector(self): #better name?
        mag_x_raw = i2c.read_int16(rd.IMU_MAG_I2C_ADDRESS, rd.IMU_MAG_X, False)
        mag_y_raw = i2c.read_int16(rd.IMU_MAG_I2C_ADDRESS, rd.IMU_MAG_Y, False)
        mag_z_raw = i2c.read_int16(rd.IMU_MAG_I2C_ADDRESS, rd.IMU_MAG_Z, False)

        scale_factor = 0.1499 # (4912 / 32760)
        mag_x = mag_x_raw * scale_factor
        mag_y = mag_y_raw * scale_factor
        mag_z = mag_z_raw * scale_factor

        return mag_x, mag_y, mag_z

    def getMagnetometerDataReadyBit(self):
        drdy = i2c.read_uint8(rd.IMU_MAG_I2C_ADDRESS, rd.IMU_MAG_ST1)
        return drdy

    def getMagnetometerOverflowBit(self):
        st2 = i2c.read_uint8(rd.IMU_MAG_I2C_ADDRESS, rd.IMU_MAG_ST2)
        hofl = st2 >> 3 & 0x01

        return hofl
        
    def getMagnetometerSt2BitM(self):
        st2 = i2c.read_uint8(rd.IMU_MAG_I2C_ADDRESS, rd.IMU_MAG_ST2)
        bitm = st2 >> 4 & 0x01

        return bitm

    def getTemperature(self):
        temperature_raw = i2c.read_uint16(rd.IMU_I2C_ADDRESS, rd.IMU_TEMPERATURE, True)
        temperature = temperature_raw / 333.87 + 21
        return temperature
        

if __name__ == '__main__':
    imu_driver = ImuDriver()
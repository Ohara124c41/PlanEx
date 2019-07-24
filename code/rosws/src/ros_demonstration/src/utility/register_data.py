#!/usr/bin/python

# ------ MOTOR ------
# NOTE: the motor registers are little endian (= low byte first)
MOTOR_I2C_ADDRESS = 0x23
# pwm
MOTOR_LEFT_PWM = 0x2
MOTOR_RIGHT_PWM = 0x4
# rpm
MOTOR_LEFT_RPM = 0x6
MOTOR_RIGHT_RPM = 0x8
# pid left
MOTOR_LEFT_P_VALUE = 0xe
MOTOR_LEFT_I_VALUE = 0x10
MOTOR_LEFT_D_VALUE = 0x12
# pid right
MOTOR_RIGHT_P_VALUE = 0x14
MOTOR_RIGHT_I_VALUE = 0x16
MOTOR_RIGHT_D_VALUE = 0x18

# ------ LIDAR ------
# NOTE: the lidar is the same device as the motor -> little endian
LIDAR_I2C_ADDRESS = 0x23

LIDAR_SENSOR_POSITION = 0x2a
LIDAR_FRONT_DISTANCE = 0x2e
LIDAR_BACK_DISTANCE = 0x30

# ------ IMU ------
# NOTE: check the datasheet what the byte order is (big / little endian)
IMU_I2C_ADDRESS = 0x68
IMU_MAG_I2C_ADDRESS = 0x0C

IMU_WHOAMI = 0x75
IMU_TEMPERATURE = 0x41
# internal i2c master interface
IMU_I2C_MASTER_INTERFACE = 0x6a
IMU_I2C_BYPASS = 0x37
# power management
IMU_POWERMANAGEMENT1 = 0x6b
IMU_POWERMANAGEMENT2 = 0x6c
# acceleration
IMU_ACCELERATION_X = 0x3b
IMU_ACCELERATION_Y = 0x3d
IMU_ACCELERATION_Z = 0x3f
# gyros
IMU_GYRO_X = 0x43
IMU_GYRO_Y = 0x45
IMU_GYRO_Z = 0x47
# magnetometer (low byte first)
IMU_MAG_X = 0x03
IMU_MAG_Y = 0x05
IMU_MAG_Z = 0x07
IMU_MAG_CNTL1 = 0x0A
IMU_MAG_ST1 = 0x02
IMU_MAG_ST2 = 0x09
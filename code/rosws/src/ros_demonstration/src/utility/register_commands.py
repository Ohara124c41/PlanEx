#!/usr/bin/python

# ------ MOTOR ------

# ------ LIDAR ------

# ------ IMU ------
IMU_RESET = 0x0
IMU_ACTIVATE = 0x0
IMU_DEACTIVATE = 0x1F # = 0b11111

IMU_DISABLE_INTERNAL_I2C_MASTER = 0x00
IMU_ACTIVATE_MAG_I2C_BYPASS = 0x02

IMU_MAG_ACTIVATE_16BIT_SINGLE = 0x11
IMU_MAG_ACTIVATE_16BIT_CONTINUOUS1 = 0x12 # (continuous1 = 8Hz, continuous2 = 100Hz sample rate)
IMU_MAG_ACTIVATE_16BIT_CONTINUOUS2 = 0x16 # (continuous1 = 8Hz, continuous2 = 100Hz sample rate)
IMU_MAG_POWERDOWN = 0x00
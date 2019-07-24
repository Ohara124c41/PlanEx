#!/usr/bin/python
import smbus
import register_data as rd

# provides functionality to communicate via I2C

bus = smbus.SMBus(1)

def read_uint8(device_address, register):
    """Reads a single byte / 8-bit value via I2C"""

    return bus.read_byte_data(device_address, register)

def read_int8(device_address, register):
    val = read_uint8(device_address, register)
    if(val >= 0x80):
        return -((0xFF - val) + 1)
    else:
        return val

def read_uint16(device_address, register, high_byte_first):
    """
    Reads two bytes via I2C and combines them to an unsigned 16-bit value.
    If high_byte_first: High byte is at the provided register address
    and the low byte one byte behind that. Vice versa otherwise.
    """
    
    high_byte = None
    low_byte = None

    if(high_byte_first):
        high_byte = bus.read_byte_data(device_address, register)
        low_byte = bus.read_byte_data(device_address, register+1)
    else:
        low_byte = bus.read_byte_data(device_address, register)
        high_byte = bus.read_byte_data(device_address, register+1)

    value = (high_byte << 8) + low_byte
    return value

def read_int16(device_address, register, high_byte_first):
    """Reads an unsigned 16-bit value via I2C and converts it to a signed value"""

    val = read_uint16(device_address, register, high_byte_first)
    if (val >= 0x8000):
        return -((0xFFFF - val) + 1)
    else:
        return val

def write8(device_address, register, value):
    """Writes a byte value (8 bit) to the given device register"""
    bus.write_byte_data(device_address, register, value)

def swap16(word):
    """Swaps a 16-bit value between big endian (high byte first) and little endian (low byte first)"""
    swapped = ((word << 8) & 0xFF00) + (word >> 8)
    return swapped

def write16(device_address, register, value, high_byte_first):
    """Writes a word value (2 byte) to the given device register"""
    if(high_byte_first):
        bus.write_word_data(device_address, register, swap16(value))
    else:
        bus.write_word_data(device_address, register, value)

if __name__ == '__main__':
    print("Reading whoAmI register of IMU (device address:" + str(hex(rd.IMU_I2C_ADDRESS)) + ", register 0x75)")
    imu_whoami = read_int8(rd.IMU_I2C_ADDRESS, rd.IMU_WHOAMI)
    print("Excpected value: 0x71")
    print("Actually read value: " + str(hex(imu_whoami)))
    print("")
    
    print("Writing to pwm register of right motor (device address: 0x23, register 0x04)")
    motor_speed = 0
    write16(rd.MOTOR_I2C_ADDRESS, rd.MOTOR_RIGHT_PWM, motor_speed, False)
    print("Reading right motor pwm register")
    right_motor_val = read_int16(rd.MOTOR_I2C_ADDRESS, rd.MOTOR_RIGHT_PWM, False)
    print("Expected value: " + str(motor_speed))
    print("Actually read value: " + str(right_motor_val))
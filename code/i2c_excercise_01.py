import smbus
from time import sleep

def bytesToInt16(highByte, lowByte):
    	# 16 bit integer is negative if the highest bit in high byte is set (high byte is greater then 0b0111 1111 or 127 or 0x7f)
	if highByte > 127:
    		# left shift high byte by 8 bits and add low byte and subtract 65536 (0x10000) to get negative number
    		return ((highByte<<8) + lowByte) - 0x10000
	else:
    		return ((highByte<<8) + lowByte)
 	

# function to read the temperature from the imu's temp sensor
def readTemp():
	# open smbus number 1 - the imu is attached there
	bus = smbus.SMBus(1)
	# read two bytes from register number 65 at i2c device address 0x68 (IMU's i2c address)
	tmpRaw = bus.read_i2c_block_data(0x68, 65, 2)
	# convert both uint8 values into one int16
	temp = bytesToInt16(tmpRaw[0], tmpRaw[1])
	# convert 16 bit temperature value into deg C float (formula from IMU's datasheet)
	temp = temp / 333.87 + 21
	return temp


if __name__ == '__main__':
    	# execute if file is called by interpreter via commandline
	print(readTemp())

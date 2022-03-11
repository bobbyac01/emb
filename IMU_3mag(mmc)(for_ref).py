#!/usr/bin/python3

import smbus
import time
from math import *

bus = smbus.SMBus(1);			# 0 for R-Pi Rev. 1, 1 for Rev. 2


#MMC5883L (Magnetometer) constants
MMC5883L_ADDRESS	=	0x30

MMC5883L_DO_X_L		=	0x00
MMC5883L_DO_X_H		=	0x01
MMC5883L_DO_Y_L		=	0x02
MMC5883L_DO_Y_H		=	0x03
MMC5883L_DO_Z_L		=	0x04
MMC5883L_DO_Z_H		=	0x05
MMC5883L_TOUT 	= 	0x06     # Output Data Registers for temperature.
MMC5883L_STAT 	= 	0x07     # Status Register.
MMC5883L_CTRL_0		=	0x08	# Control Register #0.
MMC5883L_CTRL_1		=	0x09	# Control Register #1.
MMC5883L_CTRL_2		=	0x0A	# Control Register #2.
MMC5883L_XTHS 	= 	0x0b   	# Threshold of X.
MMC5883L_YTHS 	= 	0x0b   	# Threshold of Y.
MMC5883L_ZTHS 	= 	0x0b   	# Threshold of Z.
MMC5883L_CHIP_ID 	= 	0x2f      # Chip ID register.

MMC5883_CMD_REFILL	=	0x20
MMC5883_CMD_RESET	=       0x10
MMC5883_CMD_SET	=	0x08
MMC5883_CMD_TM_M	=	0x01
MMC5883_CMD_TM_T	=	0x02

MMC5883_CMD_100HZ	=	0x00
MMC5883_CMD_200HZ	=	0x01
MMC5883_CMD_400HZ	=	0x02
MMC5883_CMD_600HZ	=	0x03

# Flags for Status Register #1.
STAT_DRDY = 0b00000001  # Data Ready.
STAT_OVL = 0b00000010   # Overflow flag.
STAT_DOR = 0b00000100   # Data skipped for reading.

# Flags for Status Register #2.
INT_ENB = 0b00000001    # Interrupt Pin Enabling.
POL_PNT = 0b01000000    # Pointer Roll-over.
SOFT_RST = 0b10000000   # Soft Reset.

# Flags for Control Register 1.
MODE_STBY = 0b00000000  # Standby mode.
MODE_CONT = 0b00000001  # Continuous read mode.
ODR_10HZ = 0b00000000   # Output Data Rate Hz.
ODR_50HZ = 0b00000100
ODR_100HZ = 0b00001000
ODR_200HZ = 0b00001100
RNG_2G = 0b00000000     # Range 2 Gauss: for magnetic-clean environments.
RNG_8G = 0b00010000     # Range 8 Gauss: for strong magnetic fields.
OSR_512 = 0b00000000    # Over Sample Rate 512: less noise, more power.
OSR_256 = 0b01000000
OSR_128 = 0b10000000
OSR_64 = 0b11000000     # Over Sample Rate 64: more noise, less power.




class IMU(object):

	def write_byte(self,adr, value):
		bus.write_byte_data(self.ADDRESS, adr, value)
	
	def read_byte(self,adr):
		return bus.read_byte_data(self.ADDRESS, adr)

	def read_word(self,adr,rf=1):
		# rf=1 Little Endian Format, rf=0 Big Endian Format
		if (rf == 1):
			low = self.read_byte(adr)
			high = self.read_byte(adr+1)
		else:
			high = self.read_byte(adr)
			low = self.read_byte(adr+1)
		val = (high << 8) + low
		return val

	def read_word_2c(self,adr,rf=1):
		val = self.read_word(adr,rf)
		if(val & (1 << 16 - 1)):
			return val - (1<<16)
		else:
			return val

class gy801(object):
	def __init__(self) :
		self.compass = MMC5883L()




class MMC5883L(IMU):
	
	ADDRESS = MMC5883L_ADDRESS

	def __init__(self) :
		#Class Properties
		self.X = None
		self.Y = None
		self.Z = None
		self.angle = None
		self.sts = 0
		self.Xoffset = 0.0
		self.Yoffset = 0.0
		self.Zoffset = 0.0
		self.angle_offset = 0.0
		
		self.scale = 0.92
		
		self.output_data_rate = ODR_10HZ
		self.output_range = RNG_2G
		self.oversampling_rate = OSR_512

		chip_id = self.read_byte(MMC5883L_CHIP_ID)
		print ("5883L chip ID : 0x%x:" % (chip_id))


#	def mode_continuous(self):
	    #"""Set the device in continuous read mode."""
		self.write_byte(REG_CONTROL_0, MMC5883_CMD_SET)  #  .
		self.write_byte(REG_CONTROL_1, MMC5883_CMD_100HZ)  #  .
		self.write_byte(REG_CONTROL_2, 0x2)  #  5Hz.
		self.write_byte(REG_CONTROL_0, MMC5883_CMD_TM_M)  #  .
		time.sleep(0.01)

	def getX(self):
		self.X = (self.read_word_2c(MMC5883L_DO_X_H) - self.Xoffset) * self.scale
		return self.X

	def getY(self):
		self.Y = (self.read_word_2c(MMC5883L_DO_Y_H) - self.Yoffset) * self.scale
		return self.Y

	def getZ(self):
		self.Z = (self.read_word_2c(MMC5883L_DO_Z_H) - self.Zoffset) * self.scale
		return self.Z
	
	def getAngle(self):
		bearing  = degrees(atan2(self.getY(), self.getX())) + self.angle_offset
		if (bearing < 0):
			bearing += 360
		bearing += self.angle_offset
		if (bearing < 0):
			bearing += 360
		if (bearing > 360):
			bearing -= 360
		self.angle = bearing
		return self.angle

	def getsts(self):
		self.sts = self.read_byte(MMC5883L_STAT)
		print ("5883L chip sts. : 0x%x:" % (self.sts))
		
		return self.sts

if __name__ == "__main__":
	# if run directly we'll just create an instance of the class and output 
	# the current readings
	
	sensors = gy801()

	compass = sensors.compass
		
	print ("\033[1;32;40mMMC5883L on address 0x%x:" % (MMC5883L_ADDRESS))
	print ("   X = %.3f " % ( compass.getX() ))
	print ("   Y = %.3f " % ( compass.getY() ))
	print ("   Z = %.3f " % ( compass.getZ() ))
	print ("   Angle = %.3f deg" % ( compass.getAngle() ))


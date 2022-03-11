#!/usr/bin/python3

import smbus
import time
from math import *

bus = smbus.SMBus(1);			# 0 for R-Pi Rev. 1, 1 for Rev. 2


#QMC5883L (Magnetometer) constants
REG_XOUT_LSB = 0x00     # Output Data Registers for magnetic sensor.
REG_XOUT_MSB = 0x01
REG_YOUT_LSB = 0x02
REG_YOUT_MSB = 0x03
REG_ZOUT_LSB = 0x04
REG_ZOUT_MSB = 0x05
REG_STATUS_1 = 0x06     # Status Register.
REG_TOUT_LSB = 0x07     # Output Data Registers for temperature.
REG_TOUT_MSB = 0x08
REG_CONTROL_1 = 0x09    # Control Register #1.
REG_CONTROL_2 = 0x0a    # Control Register #2.
REG_RST_PERIOD = 0x0b   # SET/RESET Period Register.
REG_CHIP_ID = 0x0d      # Chip ID register.

QMC5883L_ADDRESS	=	0x30

QMC5883L_DO_X_L		=	0x00
QMC5883L_DO_X_H		=	0x01
QMC5883L_DO_Y_L		=	0x02
QMC5883L_DO_Y_H		=	0x03
QMC5883L_DO_Z_L		=	0x04
QMC5883L_DO_Z_H		=	0x05
QMC5883L_STAT_1 	= 	0x06     # Status Register.
QMC5883L_TOUT_L 	= 	0x07     # Output Data Registers for temperature.
QMC5883L_TOUT_M 	= 	0x08
QMC5883L_CTRL_1		=	0x09	# Control Register #1.
QMC5883L_CTRL_2		=	0x0A	# Control Register #2.
QMC5883L_RST_PER 	= 	0x0b   	# SET/RESET Period Register.
QMC5883L_CHIP_ID 	= 	0x0d      # Chip ID register.

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
		self.compass = QMC5883L()


class QMC5883L(IMU):
	
	ADDRESS = QMC5883L_ADDRESS

	def __init__(self) :
		#Class Properties
		self.X = None
		self.Y = None
		self.Z = None
		self.angle = None
		self.Xoffset = 0.0
		self.Yoffset = 0.0
		self.Zoffset = 0.0
		self.angle_offset = 0.0
		
		self.scale = 0.92
		
		self.output_data_rate = ODR_10HZ
		self.output_range = RNG_2G
		self.oversampling_rate = OSR_512

		chip_id = self.read_byte(QMC5883L_CHIP_ID)
		if chip_id != 0xff:
		    print ("5883L chip ID : 0x%x:" % (chip_id))

		self.mode_cont = (MODE_CONT | ODR_10HZ | RNG_2G | OSR_512)
#		self.mode_stby = (MODE_STBY | ODR_10HZ | RNG_2G | OSR_64)
		self.mode_continuous()
		
	def mode_continuous(self):
	    #"""Set the device in continuous read mode."""
		self.write_byte(REG_CONTROL_2, SOFT_RST)  # Soft reset.
		self.write_byte(REG_CONTROL_2, INT_ENB)  # Disable interrupt.
		self.write_byte(REG_RST_PERIOD, 0x01)  # Define SET/RESET period.
		self.write_byte(REG_CONTROL_1, self.mode_cont)  # Set operation mode.

	def getX(self):
		self.X = (self.read_word_2c(QMC5883L_DO_X_H) - self.Xoffset) * self.scale
		return self.X

	def getY(self):
		self.Y = (self.read_word_2c(QMC5883L_DO_Y_H) - self.Yoffset) * self.scale
		return self.Y

	def getZ(self):
		self.Z = (self.read_word_2c(QMC5883L_DO_Z_H) - self.Zoffset) * self.scale
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


if __name__ == "__main__":
	# if run directly we'll just create an instance of the class and output 
	# the current readings
	
	sensors = gy801()

	compass = sensors.compass
    
	print ("\033[1;32;40mQMC5883L on address 0x%x:" % (QMC5883L_ADDRESS))
	print ("   X = %.3f " % ( compass.getX() ))
	print ("   Y = %.3f " % ( compass.getY() ))
	print ("   Z = %.3f " % ( compass.getZ() ))
	print ("   Angle = %.3f deg" % ( compass.getAngle() ))

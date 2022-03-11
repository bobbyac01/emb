#!/usr/bin/python3

import smbus
import time
from math import *

bus = smbus.SMBus(1);            # 0 for R-Pi Rev. 1, 1 for Rev. 2


# the following address is defined by datasheet
#HMC5883L (Magnetometer) constants
HMC5883L_ADDRESS        =    0x1E  # I2C address
    
HMC5883L_CRA            =    0x00  # write CRA(00), Configuration Register A
HMC5883L_CRB            =    0x01  # write CRB(01), Configuration Register B
HMC5883L_MR             =    0x02  # write Mode(02)
HMC5883L_DO_X_H         =    0x03  # Data Output
HMC5883L_DO_X_L         =    0x04
HMC5883L_DO_Z_H         =    0x05
HMC5883L_DO_Z_L         =    0x06
HMC5883L_DO_Y_H         =    0x07
HMC5883L_DO_Y_L         =    0x08



class IMU(object):

    def write_byte(self,adr, value):
        bus.write_byte_data(self.ADDRESS, adr, value)
    
    def read_byte(self,adr):
        return bus.read_byte_data(self.ADDRESS, adr)

    def read_word(self,adr,rf=1):
        # rf=1 Little Endian Format, rf=0 Big Endian Format
        if (rf == 1):
            # acc, gyro 
            low = self.read_byte(adr)
            high = self.read_byte(adr+1)
        else:
            # compass
            high = self.read_byte(adr)
            low = self.read_byte(adr+1)
        print (high)
        print (low)
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
        self.compass = HMC5883L()
        self.accel = ADXL345()



# -----------------------------------------------------

class HMC5883L(IMU):
    
    ADDRESS = HMC5883L_ADDRESS

    def __init__(self) :
        #Class Properties
        self.X = None
        self.Y = None
        self.Z = None
        self.angle = None
        self.Xoffset = 106.5
        self.Yoffset = -469.0
        self.Zoffset = -29.0
        
        # Declination Angle
        self.angle_offset = ( -1 * (4 + (32/60))) / (180 / pi)
        # Formula: (deg + (min / 60.0)) / (180 / M_PI);
        # ex: Hsinchu = Magnetic Declination: -4 deg, 32 min
        # declinationAngle = ( -1 * (4 + (32/60))) / (180 / pi)
        # http://www.magnetic-declination.com/
        
        self.scale = ?? # convert bit value(LSB) to gauss. DigitalResolution

        # Configuration Register A, write value(0x70): 0111 0000
        self.write_byte(HMC5883L_CRA, 0b????????)
        # CRA6-CRA5 = 11 -> 8 samples per measurement
        # CRA4-CRA2 = 100 -> Data Output Rate = 15Hz
        # CRA1-CRA0 = 00 -> Normal measurement configuration (Default)


        # Configuration Register B , write value(0x20): 0010 0000
        self.write_byte(HMC5883L_CRB, 0b????????)
        # CRB7-CRB5 = 001 (Gain Configuration Bits) -> Gain=1090(LSb/Gauss), default
        # ps. output range = -2048 to 2047
        
        
        # Mode Register, write value: 0000 0000
        self.write_byte(HMC5883L_MR, 0b????????)
        # MR1-MR0 = 00 (Mode Select Bits) -> Continuous-Measurement Mode.

    def getX(self):
        self.X = (self.read_word_2c(HMC5883L_DO_X_H, rf=0) - self.Xoffset) * self.scale
        return self.X

    def getY(self):
        self.Y = (self.read_word_2c(HMC5883L_DO_Y_H, rf=0) - self.Yoffset) * self.scale
        return self.Y

    def getZ(self):
        self.Z = (self.read_word_2c(HMC5883L_DO_Z_H, rf=0) - self.Zoffset) * self.scale
        return self.Z
    
    def getHeading(self):
        bearing  = degrees(atan2(self.getY(), self.getX()))

        if (bearing < 0):
            bearing += 360
        if (bearing > 360):
            bearing -= 360
        self.angle = bearing + self.angle_offset
        return self.angle



try:
    sensors = gy801()
    compass = sensors.compass

    while True:
        magx = compass.getX()
        magy = compass.getY()
        magz = compass.getZ()
    
       
        print ("Compass: " )
        print ("X = %d ," % ( magx )),
        print ("Y = %d ," % ( magy )),
        print ("Z = %d (gauss)" % ( magz ))


        
except KeyboardInterrupt:
    print("Cleanup")

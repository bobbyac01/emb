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

EARTH_GRAVITY_MS2    = 9.80665 # m/s2

ADXL345_ADDRESS    =    0x53

ADXL345_BW_RATE          =    0x2C 
ADXL345_POWER_CTL        =    0x2D 
ADXL345_DATA_FORMAT      =    0x31 
ADXL345_DATAX0           =    0x32
ADXL345_DATAY0           =    0x34
ADXL345_DATAZ0           =    0x36
ADXL345_SCALE_MULTIPLIER = 0.00390625    # G/LSP
ADXL345_BW_RATE_100HZ    = 0x0A
ADXL345_MEASURE          = 0x08 


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
        self.gyro = L3G4200D()


class L3G4200D(IMU):
    
    ADDRESS = L3G4200D_ADDRESS

    def __init__(self) :
        #Class Properties
        self.Xraw = 0.0
        self.Yraw = 0.0
        self.Zraw = 0.0
        self.X = 0.0
        self.Y = 0.0
        self.Z = 0.0
        self.Xangle = 0.0
        self.Yangle = 0.0
        self.Zangle = 0.0
        self.t0x = None
        self.t0y = None
        self.t0z = None

        # set value
        self.gain_std = 0.00875    # dps/digit
        
        self.write_byte(L3G4200D_CTRL_REG1, 0x0F)
        self.write_byte(L3G4200D_CTRL_REG4, 0x80)

        self.setCalibration()

    def setCalibration(self) :
        gyr_r = self.read_byte(L3G4200D_CTRL_REG4)
        
        self.gain = 2 ** ( gyr_r & 48 >> 4) * self.gain_std

    def getRawX(self):
        self.Xraw = self.read_word_2c(L3G4200D_OUT_X_L)
        return self.Xraw

    def getRawY(self):
        self.Yraw = self.read_word_2c(L3G4200D_OUT_Y_L)
        return self.Yraw

    def getRawZ(self):
        self.Zraw = self.read_word_2c(L3G4200D_OUT_Z_L)
        return self.Zraw

    def getX(self,plf = 1.0):
        self.X = ( self.getRawX() * self.gain ) * plf + (1.0 - plf) * self.X
        return self.X

    def getY(self,plf = 1.0):
        self.Y = ( self.getRawY() * self.gain ) * plf + (1.0 - plf) * self.Y
        return self.Y

    def getZ(self,plf = 1.0):
        self.Z = ( self.getRawZ() * self.gain ) * plf + (1.0 - plf) * self.Z
        return self.Z
    
    def getXangle(self,plf = 1.0) :
        if self.t0x is None : self.t0x = time.time()
        t1x = time.time()
        LP = t1x - self.t0x
        self.t0x = t1x
        self.Xangle = self.getX(plf) * LP
        return self.Xangle
    
    def getYangle(self,plf = 1.0) :
        if self.t0y is None : self.t0y = time.time()
        t1y = time.time()
        LP = t1y - self.t0y
        self.t0y = t1y
        self.Yangle = self.getY(plf) * LP
        return self.Yangle
    
    def getZangle(self,plf = 1.0) :
        if self.t0z is None : self.t0z = time.time()
        t1z = time.time()
        LP = t1z - self.t0z
        self.t0z = t1z
        self.Zangle = self.getZ(plf) * LP
        return self.Zangle

# -----------------------------------------------------

class ADXL345(IMU):
    
    ADDRESS = ADXL345_ADDRESS
    
    def __init__(self) :
        #Class Properties
        self.Xoffset = 0.0
        self.Yoffset = 0.0
        self.Zoffset = 0.0
        self.Xraw = 0.0
        self.Yraw = 0.0
        self.Zraw = 0.0
        self.Xg = 0.0
        self.Yg = 0.0
        self.Zg = 0.0
        self.X = 0.0
        self.Y = 0.0
        self.Z = 0.0
        self.df_value = 0b00000000    # Self test disabled, 4-wire interface
                                # Full resolution, Range = +/-2g
        self.Xcalibr = ADXL345_SCALE_MULTIPLIER
        self.Ycalibr = ADXL345_SCALE_MULTIPLIER
        self.Zcalibr = ADXL345_SCALE_MULTIPLIER

        self.write_byte(ADXL345_BW_RATE, ADXL345_BW_RATE_100HZ)    # Normal mode, Output data rate = 100 Hz
        self.write_byte(ADXL345_POWER_CTL, ADXL345_MEASURE)    # Auto Sleep disable
        self.write_byte(ADXL345_DATA_FORMAT, self.df_value)    
    
    # RAW readings in LPS
    def getRawX(self) :
        self.Xraw = self.read_word_2c(ADXL345_DATAX0)
        return self.Xraw

    def getRawY(self) :
        self.Yraw = self.read_word_2c(ADXL345_DATAY0)
        return self.Yraw
    
    def getRawZ(self) :
        self.Zraw = self.read_word_2c(ADXL345_DATAZ0)
        return self.Zraw

    # G related readings in g
    def getXg(self,plf = 1.0) :
        self.Xg = (self.getRawX() * self.Xcalibr + self.Xoffset) * plf + (1.0 - plf) * self.Xg
        return self.Xg

    def getYg(self,plf = 1.0) :
        self.Yg = (self.getRawY() * self.Ycalibr + self.Yoffset) * plf + (1.0 - plf) * self.Yg
        return self.Yg

    def getZg(self,plf = 1.0) :
        self.Zg = (self.getRawZ() * self.Zcalibr + self.Zoffset) * plf + (1.0 - plf) * self.Zg
        return self.Zg
    
    # Absolute reading in m/s2
    def getX(self,plf = 1.0) :
        self.X = self.getXg(plf) * EARTH_GRAVITY_MS2
        return self.X
    
    def getY(self,plf = 1.0) :
        self.Y = self.getYg(plf) * EARTH_GRAVITY_MS2
        return self.Y
    
    def getZ(self,plf = 1.0) :
        self.Z = self.getZg(plf) * EARTH_GRAVITY_MS2
        return self.Z

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
        
        self.scale = 0.92 # convert bit value(LSB) to gauss. DigitalResolution

        # Configuration Register A, write value(0x70): 0111 0000
        self.write_byte(HMC5883L_CRA, 0b01110000)
        # CRA6-CRA5 = 11 -> 8 samples per measurement
        # CRA4-CRA2 = 100 -> Data Output Rate = 15Hz
        # CRA1-CRA0 = 00 -> Normal measurement configuration (Default)


        # Configuration Register B , write value(0x20): 0010 0000
        self.write_byte(HMC5883L_CRB, 0b00100000)
        # CRB7-CRB5 = 001 (Gain Configuration Bits) -> Gain=1090(LSb/Gauss), default
        # ps. output range = -2048 to 2047
        
        
        # Mode Register, write value: 0000 0000
        self.write_byte(HMC5883L_MR, 0b00000000)
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
    adxl345 = sensors.accel
    gyro = sensors.gyro

    while True:
        magx = compass.getX()
        magy = compass.getY()
        magz = compass.getZ()
        adxl345.getX()
        adxl345.getY()
        adxl345.getZ()
        gyro.getXangle()
        gyro.getYangle()
        gyro.getZangle()
       
        print ("Compass: " )
        print ("X = %d ," % ( magx )),
        print ("Y = %d ," % ( magy )),
        print ("Z = %d (gauss)" % ( magz ))
        print ("ACC: ")
        print ("x = %.3f m/s2" % ( adxl345.X ))
        print ("y = %.3f m/s2" % ( adxl345.Y ))
        print ("z = %.3f m/s2" % ( adxl345.Z ))
        print ("x = %.3fG" % ( adxl345.Xg ))
        print ("y = %.3fG" % ( adxl345.Yg ))
        print ("z = %.3fG" % ( adxl345.Zg ))
        print ("x = %.3f" % ( adxl345.Xraw ))
        print ("y = %.3f" % ( adxl345.Yraw ))
        print ("z = %.3f" % ( adxl345.Zraw ))    
        print ("Gyro: ")
        print ("Xangle = %.3f deg" % ( gyro.getXangle() ))
        print ("Yangle = %.3f deg" % ( gyro.getYangle() ))
        print ("Zangle = %.3f deg" % ( gyro.getZangle() ))
        time.sleep(1)


        
except KeyboardInterrupt:
    print("Cleanup")

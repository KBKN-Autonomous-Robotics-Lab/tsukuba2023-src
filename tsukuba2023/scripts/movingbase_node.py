#!/usr/bin/env python3
import serial
import rospy
from sensor_msgs.msg import Imu
import math
import time


class movingbase_yaw:
    def __init__(self):
        self.HEADER = 6
        self.count = 0
        self.first_heading = 0             
        self.port = rospy.get_param("~port", "/dev/sensors/GNSSrover")
        self.baudrate = rospy.get_param("~baud", 19200)
        self.time_out = rospy.get_param("~time_out", 1)
        
        self.imu_msg = Imu()
        self.pub = rospy.Publisher('/movingbase_yaw', Imu, queue_size=10)
    
    def readrelposned(self):
        ackPacket=[b'\xB5',b'\x62',b'\x01',b'\x3C',b'\x00',b'\x00']
        i = 0
        payloadlength = 6    
        with serial.Serial(self.port, self.baudrate, timeout=self.time_out) as ser:
            while i < payloadlength+8:#maybe checksum
                incoming_char = ser.read()         
                if (i < 3) and (incoming_char == ackPacket[i]):
                    i += 1
                elif i == 3:
                    ackPacket[i]=incoming_char
                    i += 1           
                elif i == 4 :
                    ackPacket[i]=incoming_char
                    i += 1
                elif i == 5 :
                    ackPacket[i]=incoming_char        
                    payloadlength = int.from_bytes(ackPacket[4]+ackPacket[5], byteorder='little',signed=False)
                    i += 1
                elif (i > 5):
                    ackPacket.append(incoming_char)
                    i += 1
        
        if self.checksum(ackPacket, payloadlength) :#1
            print("checksum ok")
            nowpoint_info = self.perseheading(ackPacket)#2           
            return nowpoint_info

    def checksum(self, ackPacket, payloadlength):#1
        CK_A = 0
        CK_B = 0
        for i in range(2, payloadlength+6):
            CK_A = CK_A + int.from_bytes(ackPacket[i], byteorder='little',signed=False) 
            CK_B = CK_B + CK_A
        CK_A &= 0xff
        CK_B &= 0xff
        if (CK_A ==  int.from_bytes(ackPacket[-2], byteorder = 'little',signed=False)) and (CK_B ==  int.from_bytes(ackPacket[-1], byteorder='little',signed=False)):
            #print("ACK Received")
            return True
        else :
            print("ACK Checksum Failure:")  
            return False

    def perseheading(self,ackPacket):#2
        nowPoint = []
        #GPStime
        byteoffset = 4 +self.HEADER
        bytevalue = ackPacket[byteoffset] 
        for i in range(1,4):
            bytevalue  +=  ackPacket[byteoffset+i] 
        time = int.from_bytes(bytevalue, byteorder='little',signed=True)
        gpstime = time/1000
        nowPoint.append(gpstime)#0
        
        #Carrier solution status
        flags = int.from_bytes(ackPacket[60 + self.HEADER], byteorder='little',signed=True)
        gnssFixOK  =  flags  & (1 << 0) #gnssFixOK 
        carrSoln =  (flags   & (0b11 <<3)) >> 3 #carrSoln0:no carrier 1:float 2:fix
        nowPoint.append(gnssFixOK)#1
        nowPoint.append(carrSoln)#2
        
        #relPosHeading
        byteoffset = 24 +self.HEADER
        bytevalue = ackPacket[byteoffset] 
        for i in range(1,4):
            bytevalue  +=  ackPacket[byteoffset+i] 
        heading = int.from_bytes(bytevalue, byteorder='little',signed=True) 
        nowPoint.append(heading/100000)#3
        
        return nowPoint
        
    def movingbase_pub(self):
        nowpoint = self.readrelposned()
        
        #0~360
        heading = nowpoint[3]+90
        if heading >= 360: 
            heading -= 360
        
        if self.count == 0:
            self.first_heading=heading   
            self.count = 1
        
        relative_heading = heading - self.first_heading    
        if relative_heading < 0:
            relative_heading += 360
    
        #-180~-1
        if relative_heading>180:
            relative_heading -= 360
        
        movingbaseyaw=relative_heading*(math.pi/180)#deg>radian   
    
        self.imu_msg.header.stamp = rospy.Time.now()
    
        self.imu_msg.orientation.x = heading#0~360 deg
        self.imu_msg.orientation.y = 0
        self.imu_msg.orientation.z = movingbaseyaw #not orientation.z>>yaw
        self.imu_msg.orientation.w = 0
    
        if nowpoint[2] == 2:
            self.pub.publish(self.imu_msg)
           
        
if __name__ == "__main__":
    rospy.init_node('movingbase_yaw')
    moving_base_yaw = movingbase_yaw()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        moving_base_yaw.movingbase_pub()
        rate.sleep()

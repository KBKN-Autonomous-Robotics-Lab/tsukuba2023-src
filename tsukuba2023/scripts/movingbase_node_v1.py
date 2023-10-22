#!/usr/bin/env python
import serial
import rospy
from sensor_msgs.msg import Imu
import math
import numpy as np

class movingbase_yaw:
    def __init__(self):
        self.HEADER = 6
        self.count = 0   
        self.ackPacket=[b'\xB5',b'\x62',b'\x01',b'\x3C',b'\x00',b'\x00']
        self.payloadlength = 6                
        self.port = rospy.get_param("~port", "/dev/ttyACM2")
        self.baudrate = rospy.get_param("~baud", 115200)#19200
        self.time_out = rospy.get_param("~time_out", 0.5)#1
    
    def readrelposned(self):
        i = 0
        with serial.Serial(self.port, self.baudrate, timeout=self.time_out) as ser:
            while i < self.payloadlength+8: 
                incoming_char = ser.read()         
                if (i < 3) and (incoming_char == self.ackPacket[i]):
                    i += 1
                elif i == 3:
                    self.ackPacket[i]=incoming_char
                    i += 1           
                elif i == 4 :
                    self.ackPacket[i]=incoming_char
                    i += 1
                elif i == 5 :
                    self.ackPacket[i]=incoming_char        
                    self.payloadlength = int.from_bytes(self.ackPacket[4]+self.ackPacket[5], byteorder='little',signed=False)
                    i += 1
                elif (i > 5):
                    self.ackPacket.append(incoming_char)
                    i += 1
        if self.checksum(self.ackPacket,self.payloadlength) :#1
            self.perseheading(self.ackPacket)#2


    def checksum(self,ackPacket,payloadlength):#1
        CK_A = 0
        CK_B = 0
        for i in range(2, self.payloadlength+6):
            CK_A = CK_A + int.from_bytes(self.ackPacket[i], byteorder='little',signed=False) 
            CK_B = CK_B + CK_A
        CK_A &= 0xff
        CK_B &= 0xff
        if (CK_A ==  int.from_bytes(self.ackPacket[-2], byteorder = 'little',signed=False)) and (CK_B ==  int.from_bytes(self.ackPacket[-1], byteorder='little',signed=False)):
            #print("ACK Received")
            return True
        else :
            print("ACK Checksum Failure:")  
            return False

    def perseheading(self,ackPacket):#2
        nowPoint = [0]*7
        #GPStime
        byteoffset = 4 +self.HEADER
        bytevalue = self.ackPacket[byteoffset] 
        for i in range(1,4):
            bytevalue  +=  self.ackPacket[byteoffset+i] 
        nowPoint[1] = int.from_bytes(bytevalue, byteorder='little',signed=True)
        nowPoint[2] = self.nowPoint[1]/1000
        print("GPStime:%f sec" %float(self.nowPoint[2]))
    
        #Carrier solution status
        flags = int.from_bytes(self.ackPacket[60 + self.HEADER], byteorder='little',signed=True)
        nowPoint[3] =  flags  & (1 << 0) #gnssFixOK 
        nowPoint[4] =  (flags   & (0b11 <<3)) >> 3 #carrSoln0:no carrier 1:float 2:fix
        print("gnssFixOk:%d" %nowPoint[3])
        print("carrSoln:%d" %nowPoint[4])
    
        #relPosHeading
        byteoffset = 24 +self.HEADER
        bytevalue = self.ackPacket[byteoffset] 
        for i in range(1,4):
            bytevalue  +=  self.ackPacket[byteoffset+i] 
        nowPoint[5] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
        #print("heading:%f deg" %float(nowPoint[5]/100000))
    
        return nowPoint
        
    def movingbase_pub(self):
        rospy.init_node('movingbase_yaw')
        imu_msg = Imu()
        pub = rospy.Publisher('/movingbase_yaw', Imu, queue_size=10)
        
        rate = rospy.Rate(10)
        
        nowpoint = self.readrelposned()
        
        #0~360
        heading = nowpoint[5]/100000+90
        if heading >= 360: 
            heading -= 360
    
        if count == 0:
            first_heading=heading   
            count = 1
        
        relative_heading = heading - first_heading    
        if relative_heading < 0:
            relative_heading += 360
    
        #-180~-1
        if relative_heading>180:
            relative_heading -= 360
        
        movingbaseyaw=relative_heading*(math.pi/180)#deg>radian   

        #print("yaw:%f w"%float(np.sin(movingbaseyaw))) 
    
        imu_msg.header.stamp = rospy.get_rostime()
    
        imu_msg.orientation.x = heading#0~360 deg
        imu_msg.orientation.y = 0
        imu_msg.orientation.z = movingbaseyaw #not orientation.z>>yaw
        imu_msg.orientation.w = 0
    
        if nowpoint[4] == 2:
            pub.publish(imu_msg)
           
        rate.sleep()
        rospy.spin()
        
if __name__ == "__main__":
    moving_base_yaw = movingbase_yaw()
    moving_base_yaw.readrelposned()
    moving_base_yaw.movingbase_pub()

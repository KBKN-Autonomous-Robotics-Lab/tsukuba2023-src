import serial
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math
import numpy as np

nowPoint=[0]*10
HEADER = 6
count = 0
first_heading= 0


def readPosned():
    ackPacket=[b'\xB5',b'\x62',b'\x01',b'\x3C',b'\x00',b'\x00']
    i = 0
    payloadlength = 6
    with serial.Serial('/dev/sensors/GNSSrover', 19200, timeout=1) as ser:
        while i < payloadlength+8: 
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
    if checksum(ackPacket,payloadlength) :
        perseheading(ackPacket)


def checksum(ackPacket,payloadlength ):
    CK_A =0
    CK_B =0
    for i in range(2, payloadlength+6):
        CK_A = CK_A + int.from_bytes(ackPacket[i], byteorder='little',signed=False) 
        CK_B = CK_B +CK_A
    CK_A &=0xff
    CK_B &=0xff
    if (CK_A ==  int.from_bytes(ackPacket[-2], byteorder='little',signed=False)) and (CK_B ==  int.from_bytes(ackPacket[-1], byteorder='little',signed=False)):
        #print("ACK Received")
        return True
    else :
        print("ACK Checksum Failure:")  
        return False

def perseheading(ackPacket):
    #rospy.init_node('imu_yaw_publisher', anonymous=True)
    imu_msg = Imu()
    pub = rospy.Publisher('/movingbase_yaw', Imu, queue_size=10)
    
    rate=rospy.Rate(10)
    global count
    global first_heading
    
    #GPStime
    byteoffset =4 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    nowPoint[1] = int.from_bytes(bytevalue, byteorder='little',signed=True)
    nowPoint[2] =nowPoint[1]/1000
    print("GPStime:%f sec" %float(nowPoint[9]/1000))
    
    #Carrier solution status
    flags = int.from_bytes(ackPacket[60 + HEADER], byteorder='little',signed=True)
    nowPoint[3] =  flags  & (1 << 0) #gnssFixOK 
    nowPoint[4] =  (flags   & (0b11 <<3)) >> 3 #carrSoln0:no carrier 1:float 2:fix
    print("gnssFixOk:%d" %nowPoint[3])
    print("carrSoln:%d" %nowPoint[4])
    
    #relPosHeading
    byteoffset =24 +HEADER
    bytevalue = ackPacket[byteoffset] 
    for i in range(1,4):
        bytevalue  +=  ackPacket[byteoffset+i] 
    nowPoint[5] = int.from_bytes(bytevalue, byteorder='little',signed=True) 
    #print("heading:%f deg" %float(nowPoint[5]/100000))
    nowPoint[6]=nowPoint[5]/100000+90
    if nowPoint[6]>360: nowPoint[6]-=360
    if(count == 0):
        first_heading=nowPoint[6]-180
        count = 1
    print("robotheading:%f deg" %float(nowPoint[6]))
    print("firstheading:%f deg" %float(first_heading))
    
    absolute_heading=nowPoint[6]-180#0~360>-180~180
    relative_heading=absolute_heading-first_heading#absolute heading>relative heading
    
    movingbaseyaw=np.radians(relative_heading)/2.0#deg>radian 
    #movingbaseyaw=relative_heading*(math.pi/180)/2.0#deg>radian   

    #print("yaw:%f radian"%float(movingbaseyaw)) 
    #print("yaw:%f w"%float(np.sin(movingbaseyaw))) 
    #print("yaw:%f w"%float(np.sin(movingbaseyaw))) 
    
    imu_msg.header.stamp = nowPoint[2]#GPStime
    
    imu_msg.orientation.x = 0
    imu_msg.orientation.y = 0
    imu_msg.orientation.z = np.sin(movingbaseyaw)
    imu_msg.orientation.w = np.cos(movingbaseyaw)
           
    pub.publish(imu_msg)
    
    rate.sleep()
    #rospy.spin()
    return nowPoint

while 1:
    while not rospy.is_shutdown():
        readPosned()
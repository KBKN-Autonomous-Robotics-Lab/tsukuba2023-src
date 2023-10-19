import rospy
from sensor_msgs.msg import Imu
import math

class movingbaseNode:
    def __init__(self):
        # Initialize message and publisher
        self.sub_movingbase  = rospy.Subscriber('movingbase_yaw',Imu,self.movingbase_callback) 
        self.movingbase_msg = Imu()
        self.heading_pub = rospy.Publisher('movingbase/quo', Imu, queue_size=10)
        self.movingbase_info = []
        
    def movingbase_callback(self,data):
        self.movingbase_data = data
        self.movingbase_info.append(data.header.stamp)#0
        self.movingbase_info.append(data.orientation.z)#1 yaw
        
    def movingbase_publish_msg(self):
        rate = rospy.Rate(10)
        if len(self.movingbase_info) != 0:
            #print(self.movingbase_data.orientation.z)
            roll = 0
            pitch = 0
        
            cosRoll = math.cos(roll / 2);
            sinRoll = math.sin(roll / 2);
            cosPitch = math.cos(pitch / 2);
            sinPitch = math.sin(pitch / 2);
            cosYaw = math.cos(self.movingbase_data.orientation.z / 2);
            sinYaw = math.sin(self.movingbase_data.orientation.z  / 2);

            q0 = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
            q1 = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
            q2 = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
            q3 = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;        
            #print("orientation.x",q1,"orientation.y",q2,"orientation.z",q3,"orientation.w",q0)
            
            self.movingbase_msg.header.stamp = self.movingbase_data.header.stamp
            self.movingbase_msg.header.frame_id = "imu_link"
            self.movingbase_msg.orientation.x = q1
            self.movingbase_msg.orientation.y = q2
            self.movingbase_msg.orientation.z = q3
            self.movingbase_msg.orientation.w = q0
            self.heading_pub.publish(self.movingbase_msg)        
            self.movingbase_info.clear()
    
        
if  __name__ == "__main__":
    # init node
    rospy.init_node("movingbase")
    movingbasequo = movingbaseNode()####
    while not rospy.is_shutdown():
        movingbasequo.movingbase_publish_msg()

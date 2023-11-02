#!/usr/bin/env python3
import message_filters
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class combination_CLAS_movingbase:
    def __init__(self):
        self.sub1 = message_filters.Subscriber("/odom/gps", Odometry)
        self.sub2 = message_filters.Subscriber("movingbase/quat", Imu)#sub3.sub4--- ok
        self.mf = message_filters.ApproximateTimeSynchronizer([self.sub1, self.sub2],10,0.1,allow_headerless=True)
        self.mf.registerCallback(self.callback)
        
        self.odom_pub = rospy.Publisher("/CLAS_movingbase", Odometry, queue_size=10)
        self.odom_msg = Odometry()
        
        self.CLAS_position = None        
        self.movingbase_yaw = None   
    
    
    def callback(self,msg1,msg2):
        self.CLAS_position = msg1
        self.movingbase_yaw = msg2
    
    def pub(self):
        if self.CLAS_position is not None and self.movingbase_yaw is not None:
            #rospy.loginfo("ok")
            self.odom_msg.header.stamp = rospy.Time.now()
            self.odom_msg.header.frame_id = "odom"
            self.odom_msg.child_frame_id = "base_footprint"
            self.odom_msg.pose.pose.position.x = self.CLAS_position.pose.pose.position.x
            self.odom_msg.pose.pose.position.y = self.CLAS_position.pose.pose.position.y
            self.odom_msg.pose.pose.position.z = 0
            self.odom_msg.pose.pose.orientation.x = 0
            self.odom_msg.pose.pose.orientation.y = 0
            self.odom_msg.pose.pose.orientation.z = self.movingbase_yaw.orientation.z
            self.odom_msg.pose.pose.orientation.w = self.movingbase_yaw.orientation.w
            self.odom_msg.pose.covariance[0] = 0.0001
            self.odom_msg.pose.covariance[7] = 0.0001
            self.odom_msg.pose.covariance[14] = 0.000001
            self.odom_msg.pose.covariance[21] = 0.000001
            self.odom_msg.pose.covariance[28] = 0.000001
            self.odom_msg.pose.covariance[35] = 0.0001
            self.odom_pub.publish(self.odom_msg)
        else :
           rospy.loginfo("None")
if __name__ == "__main__":
    # init node
    rospy.init_node('CLASmovingbase')
    rate = rospy.Rate(10)
    comb = combination_CLAS_movingbase()
    while not rospy.is_shutdown():
        comb.pub()
        rate.sleep()

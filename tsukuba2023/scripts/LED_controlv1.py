#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

def main():

    rospy.init_node("publisher")

    pub = rospy.Publisher("/navBool",Bool, queue_size=10)
    navBool_msg = Bool()
    navBool_msg = True
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        pub.publish(navBool_msg)
        rate.sleep()
        
if __name__ == "__main__":
    main()

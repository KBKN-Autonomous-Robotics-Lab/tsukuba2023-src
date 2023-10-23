#!/usr/bin/env python3
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import numpy as np
import time
import math

#<rosparam param="initial_coordinate">[37, 0]</rosparam>

class GPSDataToodom:
    def __init__(self):
        self.fix = rospy.Subscriber("fix", NavSatFix, self.fix_callback)

        self.odom_pub = rospy.Publisher("/odom/gps", Odometry, queue_size=10)
        self.odom_msg = Odometry()

        self.theta = rospy.get_param(
            "~heading", 180
        )  # initial heading azithm nakaniwa 179.169287 tsukuba 291.09504
        self.initial_coordinate = rospy.get_param(
            "~initial_coordinate", [35.709918, 139.523032]
        )  # [latitude,longitude] nakaniwa 35.709918, 139.523032 tsukuba 36.082868166666664, 140.07699316666665

        self.latlon_info = []
        self.count = 0

    def fix_callback(self, data):
        self.fix_topic = data
        self.latlon_info.append(data.latitude)
        self.latlon_info.append(data.longitude)

    def conversion(self, coordinate, origin, theta):

        # set up constants
        a = 1.0 * 6378137
        f = 1.0 * 35 / 10439
        e1 = 1.0 * 734 / 8971
        e2 = 1.0 * 127 / 1547
        n = 1.0 * 35 / 20843
        a0 = 1.0 * 1
        a2 = 1.0 * 102 / 40495
        a4 = 1.0 * 1 / 378280
        a6 = 1.0 * 1 / 289634371
        a8 = 1.0 * 1 / 204422462123
        degree_to_radian = math.pi / 180

        # calculation below
        delta_latitude = coordinate[0] - origin[0]
        delta_longitude = coordinate[1] - origin[1]

        r_latitude = coordinate[0] * degree_to_radian
        r_longitude = coordinate[1] * degree_to_radian
        r_latitude_origin = origin[0] * degree_to_radian
        r_longitude_origin = origin[1] * degree_to_radian
        r_delta_latitude = delta_latitude * degree_to_radian
        r_delta_longitude = delta_longitude * degree_to_radian
        W = math.sqrt(1 - (e1 ** 2) * (math.sin(r_latitude) ** 2))
        N = a / W
        t = math.tan(r_latitude)
        ai = e2 * math.cos(r_latitude)

        S = (
            a
            * (
                a0 * r_latitude
                - a2 * math.sin(2 * r_latitude)
                + a4 * math.sin(4 * r_latitude)
                - a6 * math.sin(6 * r_latitude)
                + a8 * math.sin(8 * r_latitude)
            )
            / (1 + n)
        )
        if S != 0:
            S0 = (
                a
                * (
                    a0 * r_latitude_origin
                    - a2 * math.sin(2 * r_latitude_origin)
                    + a4 * math.sin(4 * r_latitude_origin)
                    - a6 * math.sin(6 * r_latitude_origin)
                    + a8 * math.sin(8 * r_latitude_origin)
                )
                / (1 + n)
            )
        m0 = S / S0
        B = S - S0
        y1 = (
            (r_delta_longitude ** 2)
            * N
            * math.sin(r_latitude)
            * math.cos(r_latitude)
            / 2
        )
        y2 = (
            (r_delta_longitude ** 4)
            * N
            * math.sin(r_latitude)
            * (math.cos(r_latitude) ** 3)
            * (5 - (t ** 2) + 9 * (ai ** 2) + 4 * (ai ** 4))
            / 24
        )
        y3 = (
            (r_delta_longitude ** 6)
            * N
            * math.sin(r_latitude)
            * (math.cos(r_latitude) ** 5)
            * (
                61
                - 58 * (t ** 2)
                + (t ** 4)
                + 270 * (ai ** 2)
                - 330 * (ai ** 2) * (t ** 2)
            )
            / 720
        )
        y = m0 * (B + y1 + y2 + y3)
        # rospy.logdebug("y: %f", y)

        x1 = r_delta_longitude * N * math.cos(r_latitude)
        x2 = (
            (r_delta_longitude ** 3)
            * N
            * (math.cos(r_latitude) ** 3)
            * (1 - (t ** 2) + (ai ** 2))
            / 6
        )
        x3 = (
            (r_delta_longitude ** 5)
            * N
            * (math.cos(r_latitude) ** 5)
            * (
                5
                - 18 * (t ** 2)
                + (t ** 4)
                + 14 * (ai ** 2)
                - 58 * (ai ** 2) * (t ** 2)
            )
            / 120
        )
        x = m0 * (x1 + x2 + x3)
        # rospy.logdebug("x: %f", x)

        r_theta = theta * degree_to_radian
        h_x = math.cos(r_theta) * x - math.sin(r_theta) * y
        h_y = math.sin(r_theta) * x + math.cos(r_theta) * y
        waypoint = (h_y, -h_x)
        return waypoint

    def pub(self):
        if len(self.latlon_info) != 0:
            latlon = [self.fix_topic.latitude,self.fix_topic.longitude]
            GPSxy = self.conversion(
                latlon, self.initial_coordinate, self.theta
            )
            rospy.logdebug("GPSxy: (%f, %f)", GPSxy[0], GPSxy[1])
            rospy.logdebug("LatLon info: %s", str(self.latlon_info))
            self.odom_msg.header.stamp = rospy.Time.now()
            self.odom_msg.header.frame_id = "odom"
            self.odom_msg.child_frame_id = "base_footprint"
            self.odom_msg.pose.pose.position.x = GPSxy[0]
            self.odom_msg.pose.pose.position.y = GPSxy[1]
            self.odom_msg.pose.pose.position.z = 0
            # status.status = 2 is an analytical Fix solution based on the reference station. fix 2 nonfix -1
            if self.fix_topic.status.status == 2:
                self.odom_pub.publish(self.odom_msg)
            #self.latlon_info.clear()


if __name__ == "__main__":
    # init node
    rospy.init_node("gps_data_acquisition")
    rate = rospy.Rate(10)
    gtodom = GPSDataToodom()
    while not rospy.is_shutdown():#resalt same 10/22
        gtodom.pub()
        rate.sleep()

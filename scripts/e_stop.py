#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector', anonymous=False)
        self.sub_ls = rospy.Subscriber(
            '/scan',
            LaserScan,
            callback=self.callback
        )
        self.pub_flag = rospy.Publisher(
            "/lidar_flag",
            Bool,
            queue_size=3
        )

    def callback(self, _ls):
        for distance in _ls.ranges[340:360]:
            if distance > 0 and distance < 0.9:
                self.pub_flag.publish(False)
                return
        self.pub_flag.publish(True)

def run():
    od = ObjectDetector()
    rospy.spin()

if __name__ == "__main__":
    run()

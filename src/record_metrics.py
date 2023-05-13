#!/usr/bin/env python3


import rospy

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Empty


class metrics:
    def __init__(self):
        rospy.init_node("camera_info_publisher")

        self.header_store = {}

        rospy.Subscriber('/ar_camera/image_compressed', CompressedImage, self.start_timer)
        rospy.Subscriber('/ar_camera/aligned/depth', Image, self.stop_timer)

        self.acked = rospy.Publisher('/acked', Empty)
        rospy.spin()


    def start_timer(self, data):
        time = rospy.Time.now() - data.header.stamp
        print(f"Time for compressed image propogation: {rospy.Time.to_sec(time)}")

        self.start_time = rospy.Time.now() 

    def stop_timer(self):
        time = (rospy.Time.now() - self.start_time)
        print(f"Time through img proc stack: {rospy.Time.to_sec(time)}")
        
        msg = Empty()
        self.acked.publish(msg)

if __name__ == '__main__':
    metrics()
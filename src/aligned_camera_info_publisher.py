#!/usr/bin/env python3

'''
https://www.geeksforgeeks.org/reading-and-writing-json-to-a-file-in-python/
https://github.com/DFKI-NI/rospy_message_converter
'''

import rospy

from sensor_msgs.msg import CameraInfo

from sensor_msgs.srv import SetCameraInfo

from rospy_message_converter import json_message_converter

import json


class camera_info_publisher:
    def __init__(self):
        self.info_pub = rospy.Publisher(
            "/ar_camera/aligned/camera_info", CameraInfo, queue_size=1)

        self.msg = CameraInfo()

        self.readFromJson()


    def publish(self, data):
        self.msg.header.stamp = data
        self.msg.header.frame_id = 'base_link'

        self.info_pub.publish(self.msg)

    def readFromJson(self):
        with open('/home/brennan/catkin_ws/src/ar_slam/src/info.json', 'r') as openfile:
            data = json.load(openfile)
            self.msg = json_message_converter.convert_json_to_ros_message(
                'sensor_msgs/CameraInfo', json.dumps(data))




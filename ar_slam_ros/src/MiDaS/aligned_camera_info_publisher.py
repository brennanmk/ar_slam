#!/usr/bin/env python3

'''
https://www.geeksforgeeks.org/reading-and-writing-json-to-a-file-in-python/
https://github.com/DFKI-NI/rospy_message_converter
'''

import rospy

from sensor_msgs.msg import CameraInfo

from rospy_message_converter import json_message_converter

import json


class aligned_camera_info_publisher:
    '''
    aligned_camera_info_publisher is very similar to the camera_info_publisher node

    This node is called by midas_ros to publish a camera info message that has the same time stamp as the depth image
    '''
    def __init__(self):
        self.info_pub = rospy.Publisher(
            "/ar_camera/aligned/camera_info", CameraInfo, queue_size=1)

        self.msg = CameraInfo()

        self.readFromJson()


    def publish(self, data):
        '''
        Publishes camera info message with the same time stamp as the depth image
        '''

        self.msg.header.stamp = data
        self.msg.header.frame_id = 'base_link'

        self.info_pub.publish(self.msg)

    def readFromJson(self):
        '''
        reads camera info message from json file
        '''

        with open('/home/brennan/catkin_ws/src/ar_slam/ar_slam_ros/src/info.json', 'r') as openfile:
            data = json.load(openfile)
            self.msg = json_message_converter.convert_json_to_ros_message(
                'sensor_msgs/CameraInfo', json.dumps(data))




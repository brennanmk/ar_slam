#!/usr/bin/env python3

'''
Brennan Miller-Klugman

https://www.geeksforgeeks.org/reading-and-writing-json-to-a-file-in-python/
https://github.com/DFKI-NI/rospy_message_converter
'''

import rospy

from sensor_msgs.msg import CameraInfo

from sensor_msgs.srv import SetCameraInfo

from rospy_message_converter import json_message_converter

import json


class camera_info_publisher:
    '''
    camera_info_publisher node reads a camera info message that is saved in a json file and publishes a latched topic
    The camera info message can be updated by calling the /set_camera_info service

    The camera info message is used for image_proc 
    '''

    def __init__(self):
        rospy.init_node("camera_info_publisher")
        self.info_pub = rospy.Publisher(
            "/ar_camera/camera_info", CameraInfo, queue_size=1, latch=True)

        self.msg = CameraInfo()
        self.svc = rospy.Service('/set_camera_info', SetCameraInfo,
                                 self.updateCameraInfo)

        self.readFromJson()
        self.publish()
        rospy.spin()

    def publish(self):
        '''
        Publishes latched camera info message
        '''

        self.info_pub.publish(self.msg)

    def readFromJson(self):
        '''
        Use rospy_message_converter to convert json to ros message
        '''

        with open('/home/brennan/catkin_ws/src/ar_slam/ar_slam_ros/src/info.json', 'r') as openfile:
            data = json.load(openfile)
            self.msg = json_message_converter.convert_json_to_ros_message(
                'sensor_msgs/CameraInfo', json.dumps(data))

    def updateCameraInfo(self, data):
        '''
        If the service is called, update the camera info message json file

        The node will need to be restarted for the new camera info message to be utilized
        '''

        self.msg = data.camera_info
        json_data = json_message_converter.convert_ros_message_to_json(
            data.camera_info)
        with open("info.json", "w") as outfile:
            outfile.write(json_data)


if __name__ == '__main__':
    camera_info_publisher()

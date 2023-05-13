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
        self.info_pub.publish(self.msg)

    def readFromJson(self):
        with open('/home/brennan/catkin_ws/src/ar_slam/src/info.json', 'r') as openfile:
            data = json.load(openfile)
            self.msg = json_message_converter.convert_json_to_ros_message(
                'sensor_msgs/CameraInfo', json.dumps(data))

    def updateCameraInfo(self, data):
        self.msg = data.camera_info
        json_data = json_message_converter.convert_ros_message_to_json(
            data.camera_info)
        with open("info.json", "w") as outfile:
            outfile.write(json_data)

if __name__ == '__main__':
    camera_info_publisher()
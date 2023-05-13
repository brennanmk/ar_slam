#!/usr/bin/env python3


import rospy

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Empty


class metrics:
    """
    Metrics class is used to collect metrics about the ar_slam stack

    This class prints:
        An average of the time taken to propagate from the mobile device to the the device this node runs on
        The average time taken to propogate across the ar_slam stack

    The longer this node runs, the more accurate the results
    """

    def __init__(self):
        rospy.init_node("metrics")

        self.image_prop_avg = 0
        self.total_stack = 0
        self.message_count = 0

        rospy.Subscriber('/ar_camera/image_compressed',
                         CompressedImage, self.start_timer)
        rospy.Subscriber('/ar_camera/aligned/depth', Image, self.stop_timer)

        self.acked = rospy.Publisher('/acked', Empty)
        rospy.spin()

    def start_timer(self, data):
        """
        start_timer prints out the time that was taken to propagate the image from the mobile device to the the device this node runs on
        Additionally, records the time that the compressed image enters the ar_slam stack
        """
        curr_time = rospy.Time.now()
        self.total_stack += 1

        print(f"################# {self.message_count} #################")
        self.image_prop_avg += (curr_time - data.header.stamp)
        print(
            f"Time for compressed image propogation: {self.image_prop_avg / self.message_count}")

        self.start_time = curr_time

    def stop_timer(self):
        """
        Prints out the time taken for image to propogate across ar_slam stack
        Also acknoledges that the image has propogated through the stack and publishes an empty message signaling unity to send a new image
        """
        self.total_stack += (rospy.Time.now() - self.start_time)
        print(
            f"Time through img proc stack: {self.total_stack / self.message_count}")

        msg = Empty()
        self.acked.publish(msg)


if __name__ == '__main__':
    metrics()

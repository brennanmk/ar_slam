#!/usr/bin/env python3

#https://github.com/NVIDIA-AI-IOT/ros2_torch2trt_examples/tree/main/ros2_monocular_depth/ros2_monocular_depth 

import torch
import cv2


import numpy as np

from MiDaS.midas.model_loader import default_models, load_model

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from skimage.util import img_as_float
from aligned_camera_info_publisher import camera_info_publisher

class midas_ros:
    def __init__(self):
        rospy.init_node('midas_ros')
        self.first_execution = True

        self.camera_info_pub = camera_info_publisher()

        torch.backends.cudnn.enabled = True
        torch.backends.cudnn.benchmark = True

        self.bridge = CvBridge()
        
        self.depth_pub = rospy.Publisher(
            "/ar_camera/aligned/depth", Image, queue_size=1)

        self.aligned_image_pub = rospy.Publisher(
            "/ar_camera/aligned/image_raw", Image, queue_size=1)

        self.run(default_models["dpt_beit_large_512"])

    def process(self, device, model, model_type, image, input_size, target_size, optimize, use_camera):
        """
        Run the inference and interpolate.

        Args:
            device (torch.device): the torch device used
            model: the model used for inference
            model_type: the type of the model
            image: the image fed into the neural network
            input_size: the size (width, height) of the neural network input (for OpenVINO)
            target_size: the size (width, height) the neural network output is interpolated to
            optimize: optimize the model to half-floats on CUDA?
            use_camera: is the camera used?

        Returns:
            the prediction
        """

        sample = torch.from_numpy(image).to(device).unsqueeze(0)

        if self.first_execution or not use_camera:
            height, width = sample.shape[2:]
            print(
                f"    Input resized to {width}x{height} before entering the encoder")
            self.first_execution = False

        prediction = model.forward(sample)
        prediction = (
            torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=target_size[::-1],
                mode="bicubic",
                align_corners=False,
            )
            .squeeze()
            .cpu()
            .numpy()
        )

        return prediction

    def run(self, model_path, model_type="dpt_beit_large_512", optimize=False, height=None,
            square=False, grayscale=False):
        """Run MonoDepthNN to compute depth maps.

        Args:
            input_path (str): path to input folder
            output_path (str): path to output folder
            model_path (str): path to saved model
            model_type (str): the model type
            optimize (bool): optimize the model to half-floats on CUDA?
            side (bool): RGB and depth side by side in output images?
            height (int): inference encoder image height
            square (bool): resize to a square resolution?
            grayscale (bool): use a grayscale colormap?
        """
        print("Initialize")

        # select device
        device = torch.device("cuda")

        self.model, self.transform, self.net_w, self.net_h = load_model(
            device, model_path, model_type, optimize, height, square)

        with torch.no_grad():
            while True:
                data = rospy.wait_for_message("/ar_camera/image_rect_color", Image)
                original_image_rgb = self.bridge.imgmsg_to_cv2(
                    data, desired_encoding='passthrough')

                image = self.transform({"image": original_image_rgb/255})["image"]

                depth = self.process(device, self.model, model_type, image, (self.net_w, self.net_h),
                                     original_image_rgb.shape[1::-1], optimize, True)

                # Normalize prediction
                depth_min = depth.min()
                depth_max = depth.max()
                normalized_depth = 255 * (depth - depth_min) / (depth_max - depth_min)
                
                depth = normalized_depth.astype('uint16')
                image_message = self.bridge.cv2_to_imgmsg(
                    depth, encoding="passthrough")
                
                time_stamp = rospy.Time.now()
                image_message.header.stamp = time_stamp
                image_message.header.frame_id = 'base_link'

                data.header.stamp = time_stamp
                data.header.frame_id = 'base_link'

                self.camera_info_pub.publish(time_stamp)
                self.depth_pub.publish(image_message)
                self.aligned_image_pub.publish(data)


if __name__ == "__main__":
    midas_ros()

# ar_slam

## Setup

### Unity

Install Unity 2021.3.13f1 from https://unity.com/download

Once installed, open the ar_slam_unity from Unity Hub.

Once open, the IP address of your ROS_TCP_Endpoint can be adjusted under the parameters of the ROS game object
available in the Cam scene.

Adding a depthImage component to the ROS game object and building the application will result in images from ARCores
depth API to be published

Adding an BenchmarkImageCollection component to the ROS game object and building the application will launch the application in
benchmark mode.

Adding an ImageCollection component to the ROS game object and building the application will launch the application normal mode

The aforementioned components include multiple parameters including image height and width which allows the resolution of the collected image
to be adjusted.

### MiDaS

To install MiDaS navigate to https://github.com/isl-org/MiDaS and clone the contents of the repository into
ar_slam/ar_slam_ros/src/MiDaS.

From here, the rest of the MiDaS setup process outlined in the README can be followed. Note that all dependencies outlined in the conda environment.yaml
file will need to be installed using pip.

Once completed, download the following weights:
https://github.com/isl-org/MiDaS/releases/download/v3_1/dpt_beit_large_512.pt
https://github.com/isl-org/MiDaS/releases/download/v3_1/dpt_swin2_large_384.pt

And move them into ar_slam/ar_slam_ros/src/MiDaS/weights.

### ROS

After cloning repo, make sure to change the absolute path of the info.json file in the camera_info_publisher and aligned_camera_info_publisher nodes

Make sure to calibrate your mobile device with http://wiki.ros.org/camera_calibration.

To run the image processing and mapping stack the following commands need to be ran
`rosrun image_transport republish compressed in:=/ar_camera raw out:=/ar_camera/raw_image`
`roslaunch ar_slam ar_slam.launch tcp_ip:=your_ip`
`rosrun ar_slam midas_ros.py`
`ROS_NAMESPACE=ar_camera rosrun image_proc image_proc`

To run in benchmark mode, also run
`rosrun ar_slam record_metrics.py`

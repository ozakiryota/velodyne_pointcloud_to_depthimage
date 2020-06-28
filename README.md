# velodyne_pointcloud_to_depthimage
Generate depth images from velodyne point cloud.
## DEMO
![velodyne_pointcloud_to_depthimage](https://user-images.githubusercontent.com/37431972/85836136-21b85d80-b7d1-11ea-9797-9ccabb597f75.png)
## Features
* Subscribe:
  * "/velodyne_points" (sensor_msgs::PointCloud2)
* Publish:
  * "/depth_image/64fc1" (sensor_msgs::Image) (64FC1)
  * "/depth_image/16uc1" (sensor_msgs::Image) (mono16)
  * "/depth_image/8uc1" (sensor_msgs::Image) (mono8)
* Save (optional):
  * "\*\*\*.yaml" (CV_64FC1\<double\>)
  * "\*\*\*/\*\*\*.jpg" (CV_16UC1\<unsigned short\>)
  * "\*\*\*/\*\*\*.jpg" (CV_8UC1\<unsigned char\>)
## Requirement
* ROS
* PCL
* OpenCV
## Installation
```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/ozakiryota/velodyne_pointcloud_to_depthimage
$ cd ~/catkin_ws
$ catkin_make
```
## Usage
### Edit the launch file
* Set your bag file path.
* Set parameters
  * {num_ring}  
  number of velodyne sensor's layers
  * {points_per_ring}  
  number of points per ring
  * {depth_resolution}  
  resolution of depth (for "mono16")  
  (ex. when depth_resolution is 0.1, 1.234 m is registered as 12 in grayscale)
  * {max_range}  
  max range of laser (for "mono8")
  * {save_limit}  
  maximum number for saving jpg images  
  (if you don't want to save any images, set as save_limit=-1)
  * {save_root_path}  
  path to directory for saving whole datas
  * {save_jpgdir16u_name}  
  name of directory for saving "mono16" images
  * {save_jpgdir8u_name}  
  name of directory for saving "mono8" images
  * {save_img_name}  
  name of saved image file  
  (images are saved as "{save_root_path}/{save_jpgdir16u_name}/{save_img_name}1.jpg", "{save_root_path}/{save_jpgdir16u_name}/{save_img_name}2.jpg",,,)
  * {save_yaml_name}  
  name of saved yaml file  
  (yaml file is saved as  "{save_root_path}/{save_yaml_name}.yml")
### Launch
```bash
$ roslaunch velodyne_pointcloud_to_depthimage velodyne_pointcloud_to_depthimage.launch
```
## Note
### Data type
* 64FC1\<double\>: 2.2250738585072014e-308 ~ 1.7976931348623158e+308	
* mono16\<unsigned short\>: 0 ~ 65535
* mono8\<unsigned char\: 0 ~ 255
### ROS -> CV
#### C++
    #include <sensor_msgs/PointCloud2.h>
    #include <opencv2/opencv.hpp>
    #include <cv_bridge/cv_bridge.h>
    sensor_msgs::ImagePtr ros_msg;
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(ros_msg, ros_msg->encoding);
    cv::Mat cv_mat = cv_ptr->image
#### Python
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    ros_img = Image()
    bridge = CvBridge()
    cv_img = bridge.imgmsg_to_cv2(ros_img, ros_img.encoding)

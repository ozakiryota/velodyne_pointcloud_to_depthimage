# velodyne_pointcloud_to_depthimage
Generate depth images from velodyne point cloud.
## DEMO
Preparing...
## Features
* Input: sensor_msgs::PointCloud2
* Output: sensor_msgs::ImagePtr (mono16)
* Save (optional): CV_64FC1 (double) in jpg
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
  * num_ring: number of velodyne sensor's layers
  * points_per_ring: number of points per ring
  * depth_resolution: resolution of depth  
  (ex. when depth_resolution is 0.1, 1.234 m is registered as 12 in grayscale)
  * save_limit: maximum number for saving jpg images  
  (if you don't want to save any images, set as save_limit=-1)
  * save_dir_path: path to directory for saving images
  * save_img_name: name of saved image  
  (images are save as save_dir_path/save_img_name1.jpg, save_dir_path/save_img_name2.jpg,,,)
### Launch
```bash
$ roslaunch velodyne_pointcloud_to_depthimage velodyne_pointcloud_to_depthimage.launch
```

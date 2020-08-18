# FROM ros:kinetic
FROM osrf/ros:kinetic-desktop-full

########## nvidia-docker1 hooks ##########
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}
########## BASIS ##########
RUN apt-get update && apt-get install -y \
	vim \
	wget \
	unzip \
	git \
	build-essential
########## ROS setup ##########
RUN mkdir -p /home/ros_catkin_ws/src && \
	cd /home/ros_catkin_ws/src && \
	/bin/bash -c "source /opt/ros/kinetic/setup.bash; catkin_init_workspace" && \
	cd /home/ros_catkin_ws && \
	/bin/bash -c "source /opt/ros/kinetic/setup.bash; catkin_make" && \
	echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc && \
	echo "source /home/ros_catkin_ws/devel/setup.bash" >> ~/.bashrc && \
	echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:/home/ros_catkin_ws" >> ~/.bashrc && \
	echo "export ROS_WORKSPACE=/home/ros_catkin_ws" >> ~/.bashrc && \
	echo "function cmk(){\n lastpwd=\$OLDPWD \n cpath=\$(pwd) \n cd /home/ros_catkin_ws \n catkin_make \$@ \n cd \$cpath \n	OLDPWD=\$lastpwd \n}" >> ~/.bashrc
########## pcl 1.9.1 ##########
RUN	mkdir -p /home/pcl_ws && \
	cd /home/pcl_ws && \
	wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.9.1.tar.gz && \
	tar -zxvf pcl-1.9.1.tar.gz && \
	cd pcl-pcl-1.9.1 && \
	mkdir build && \
	cd build && \
	cmake .. && \
	make -j $(nproc --all) && \
	make -j $(nproc --all) install
########## velodyne_pointcloud_to_depthimage ##########
RUN	cd /home/ros_catkin_ws/src && \
	git clone https://github.com/ozakiryota/velodyne_pointcloud_to_depthimage && \
	cd /home/ros_catkin_ws && \
	/bin/bash -c "source /opt/ros/kinetic/setup.bash; catkin_make"
######### Initial position ##########
WORKDIR /home/ros_catkin_ws/src/velodyne_pointcloud_to_depthimage

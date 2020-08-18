#!/bin/bash

image_name="velodyne_pointcloud_to_depthimage"
tag_name="nvidia_docker1"

docker build . \
	-t $image_name:$tag_name

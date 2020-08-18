#!/bin/bash

image_name="velodyne_pointcloud_to_depthimage"
tag_name="docker"

docker run -it --rm \
	--net=host \
	$image_name:$tag_name

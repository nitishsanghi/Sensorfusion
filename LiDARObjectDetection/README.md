# LiDAR Object Detection
In this project a pipeline implemented to process LiDAR point cloud data using PCL data structures and custom implemented segmentation and clustering functions. The segmentation is done using RANSAC algorithm to segment point clouds of objects from those belonging to the ground/road followed by clustering of object point cloud in to object clusters using kd-tree data structure and a euclidean distance metric. Then bounding boxes for object clusters are found for representation.
![Alt Text](https://media.giphy.com/media/vIpGW8I6sYJRHKXeUJ/giphy.gif)


## Installation

### Linux Ubuntu 16

Install PCL, C++

The link here is very helpful, 
https://larrylisky.com/2014/03/03/installing-pcl-on-ubuntu/

A few updates to the instructions above were needed.

* libvtk needed to be updated to libvtk6-dev instead of (libvtk5-dev). The linker was having trouble locating libvtk5-dev while building, but this might not be a problem for everyone.

* BUILD_visualization needed to be manually turned on, this link shows you how to do that,
http://www.pointclouds.org/documentation/tutorials/building_pcl.php


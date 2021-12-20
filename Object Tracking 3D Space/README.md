# 3D Object Tracking

In this project, I developed a way to match 3D objects over time by using keypoint correspondences and then I computed two estimates for TTC (Time To Collision) based on Lidar measurements and camera image keypoints. The objects in each image are identified using YOLO deep-learning framework which generates bounding boxes with a certain confidence level. In the pipeline the bounding boxes in each image frame are matched to track objects from frame to frame. The illustration below shows bounding boxes identifying objects in a sequence of images.

![Alt Text](https://media.giphy.com/media/EfT9Da1keQ2CwKBXjs/giphy.gif)

The objective of this project is to estimate the Time-To-Collision (TTC) fromt the preceding vehicle. Estimates are made from LiDAR data points and by tracking objects in camera images. In order to do so and make computation simpler the pipeline only focuses on the preceding vehicle by processing LiDAR points and Camera image keypoints present within a ROI around the preceding vehicle. In the illustration below TTC estimates are overlayed on image frames with bounding box around the preceding vehicle and LiDAR point projected in to the camera frame within the bounding boxes.

![Alt Text](https://media.giphy.com/media/APzLCb6rjOgc8ssqUZ/giphy.gif)

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.

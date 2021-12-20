# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

Objective and Overview
In this "Feature tracking" project, I implemented a few detectors, descriptors, and matching algorithms using OPENCV C++ libraries. The project consists of three main components: First, keypoint detection in which a feature detection algorithm identifies probable keypoint features. Second is descriptor extraction for keypoints found by the detectors. And third and final component of descriptor matching between image frames. List of detectors, descriptors, and matching functions implemented below.

* Keypoint Detectors: SHI-TOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT
* Descriptor Extraction: BRISK, BRIEF, FREAK, ORB, AKAZE, SIFT
* Matching: Brute-force and FLANN approach couple with distance ratio filter

Both traditional and modern/binary detectors/descriptors have been evaluated in this project. The gif below is made up of consecutive image frames which were processed through a FAST keypoint detector.
![](<div style="width:100%;height:0;padding-bottom:30%;position:relative;"><iframe src="https://giphy.com/embed/4tRk2t8sL0XwIKuNby" width="100%" height="100%" style="position:absolute" frameBorder="0" class="giphy-embed" allowFullScreen></iframe></div><p><a href="https://giphy.com/gifs/4tRk2t8sL0XwIKuNby">via GIPHY</a></p>)



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
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

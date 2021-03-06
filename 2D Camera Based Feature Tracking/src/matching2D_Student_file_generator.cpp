#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{   //cout << "Matching function 1" << endl;

    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;
    //cout << "Matching function 2" << endl;
    if (matcherType.compare("MAT_BF") == 0)
    {//cout << "Matching function 3" << endl;
        int normType = cv::NORM_HAMMING;
	if (descriptorType.compare("DES_HOG") == 0){
		normType = cv::NORM_L2;	
	}
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        matcher = cv::FlannBasedMatcher::create();
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)
	//cout << "Matching function 4" << endl;
	std::vector<std::vector<cv::DMatch>> knnmatches;
        matcher->knnMatch(descSource, descRef,knnmatches, 2);
	//cout << "Matching function 5" << endl;
	for(auto it = knnmatches.begin(); it<knnmatches.end(); it++){
		
		if((*it)[0].distance < 0.8*(*it)[1].distance){
			matches.push_back((*it)[0]);		
		}	
	}
    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType, double &time)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(30, 3, 1.0f);
    }
    else if (descriptorType.compare("BRIEF") == 0)
    {
	int bytes = 32;
	bool use_orientation = false;
	extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(32, false);
    }
    else if (descriptorType.compare("ORB") == 0)
    {

        extractor = cv::ORB::create(10000, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
    }
    else if (descriptorType.compare("FREAK") == 0)
    {
	bool orientationNormalized = true;
	bool scaleNormalized = true;
	float patternScale = 22.0f;
	int nOctaves = 4;
	const std::vector< int > & 	selectedPairs = std::vector< int >();
	extractor = cv::xfeatures2d::FREAK::create(true, true, 22.0f, 4, selectedPairs);
    }
    else if (descriptorType.compare("AKAZE") == 0)
    {
    	int descriptor_type = 3;
	int descriptor_size = 0;
	int descriptor_channels = 3;
	float threshold = 0.001f;
	int nOctaves = 4;
	int nOctaveLayers = 4;
	int diffusivity = cv::KAZE::DIFF_PM_G2;
    	extractor = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.001f, 4, 4, cv::KAZE::DIFF_PM_G2);
    }
    else
    {
    	int nfeatures = 0;
	int nOctaveLayers = 3;
	double	contrastThreshold = 0.04;
	double edgeThreshold = 10;
	double sigma = 1.6;
    	extractor = cv::xfeatures2d::SIFT::create(0, 3, 0.04, 10, 1.6);
    }


    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    time = 1000*t;
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, double &time, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    time = 1000*t;
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsHarris(vector<cv::KeyPoint> &keypoints, cv::Mat &img, double &time, bool bVis)
{
    // Detector parameters
int blockSize = 2; // for every pixel, a blockSize ?? blockSize neighborhood is considered
int apertureSize = 3; // aperture parameter for Sobel operator (must be odd)
int minResponse = 95; // minimum value for a corner in the 8bit scaled response matrix
double k = 0.04; // Harris parameter
auto t = (double)cv::getTickCount();
// Detect Harris corners and normalize output
cv::Mat dst, dst_norm, dst_norm_scaled;
dst = cv::Mat::zeros(img.size(), CV_32FC1);
cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
cv::normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
cv::convertScaleAbs( dst_norm, dst_norm_scaled );


    // add corners to result vector
    for( int i = 0; i < dst_norm.rows ; i++ )
    {
        for( int j = 0; j < dst_norm.cols; j++ )
        {
            if( (int) dst_norm.at<float>(i,j) > minResponse )
            {
        	cv::KeyPoint newKeyPoint;
        	newKeyPoint.pt = cv::Point2f(j,i);;
        	newKeyPoint.size = blockSize;
		//newKeyPoint.size = 2*apertureSize;
        	keypoints.push_back(newKeyPoint);
                
            }
        }
    }
    
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    time = 1000*t;
    cout << "Harris detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Harris Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsModern(vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, double &time, bool bVis)
{

    // Apply corner detection
    double t = (double)cv::getTickCount();

    if (detectorType.compare("FAST") == 0) 
    {
	
	int threshold = 10;
	bool nonmaxSuppression = true;
	int type = cv::FastFeatureDetector::TYPE_9_16;
	cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(10, true, cv::FastFeatureDetector::TYPE_9_16);
	detector->detect(img, keypoints);
    }
    else if (detectorType.compare("BRISK") == 0)
    {
	int thresh = 30;
	int octaves = 3;
	float patternScale = 1.0f;
	cv::Ptr<cv::BRISK> detector = cv::BRISK::create(30, 4, 1.0f);
	detector->detect(img, keypoints);
    }
    else if (detectorType.compare("ORB") == 0)
    {
	int nfeatures = 10000;
	float scaleFactor = 1.2f;
	int nlevels = 8;
	int edgeThreshold = 31;
	int firstLevel = 0;
	int WTA_K = 2;
	int scoreType = cv::ORB::HARRIS_SCORE;
	int patchSize = 31;
	int fastThreshold = 20;
	cv::Ptr<cv::ORB> detector = cv::ORB::create(10000, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
	detector->detect(img, keypoints);
    }
    else if (detectorType.compare("AKAZE") == 0)
    {
	int descriptor_type = cv::AKAZE::DESCRIPTOR_MLDB;
	int descriptor_size = 0;
	int descriptor_channels = 3;
	float threshold = 0.001f;
	int nOctaves = 4;
	int nOctaveLayers = 4;
	int diffusivity = cv::KAZE::DIFF_PM_G2;
	cv::Ptr<cv::AKAZE> detector = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.001f, 4, 4, cv::KAZE::DIFF_PM_G2);
	detector->detect(img, keypoints);
    }
    else
    {
	int nfeatures = 0;
	int nOctaveLayers = 3;
	double	contrastThreshold = 0.04;
	double edgeThreshold = 10;
	double sigma = 1.6;
	cv::Ptr<cv::xfeatures2d::SIFT> detector = cv::xfeatures2d::SIFT::create(0, 3, 0.04, 10, 1.6);
	detector->detect(img, keypoints);
    }

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    time = 1000*t;
    cout << detectorType <<" detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

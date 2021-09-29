/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
          // visualize results

    /* MAIN LOOP OVER ALL IMAGES */
    vector<string> detectorTypes{"SHITOMASI","HARRIS","FAST","BRISK","ORB","AKAZE","SIFT"};
    vector<string> descriptorTypes{"BRISK","BRIEF","ORB","FREAK","AKAZE","SIFT"};
    //vector<vector<int>> detectorKeypoints;
    //vector<vector<int>> pvdetectorKeypoints;
    //vector<vector<double>> timeKeypoints;
    //vector<vector<double>> timeDescriptors;
    //vector<vector<int>> matchesKeypoints;
    double time;
    for (auto it1 = detectorTypes.begin(); it1 < detectorTypes.end(); it1++)
    {
    	// Save results
    	string imgResultFolder = dataPath + "imagesResults/";
        vector<int> keypointImages;
        vector<int> pvkeypointImages;
	vector<double> timeImageKeypoints;
	vector<double> timeImageDescriptors;
        vector<int> matchesImage;
	string resultfile = imgResultFolder + (*it1) + "/"+ (*it1)+ "_"+"results.txt";
	ofstream file(resultfile);
    for (auto it2 = descriptorTypes.begin(); it2 < descriptorTypes.end(); it2++)
    { 
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;
    int imgPairCounter = 1;  
    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    { cout << "Entering Image loop" << endl;
        /* LOAD IMAGE INTO BUFFER */
	if ((*it1).compare("AKAZE") != 0 && (*it2).compare("AKAZE") == 0)
	{
	    continue;	
	}
	if ((*it1).compare("SIFT") == 0 && (*it2).compare("ORB") == 0)
	{
	    continue;
	}
        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        //imgGray.convertTo(imgGray, CV_8U);
        frame.cameraImg = imgGray;

	if (dataBuffer.size() >= dataBufferSize){
		dataBuffer.erase(dataBuffer.begin());	
	}
        dataBuffer.push_back(frame);
	cout << " Buffer size " << dataBuffer.size() << endl;

        //// EOF STUDENT ASSIGNMENT
        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
	vector<cv::KeyPoint> keypointsrect;
	string detectorType = (*it1);
	cout << "Detector Type : " << detectorType << endl;
	
        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray, time, false);
        }
        else if (detectorType.compare("HARRIS") == 0)
        {
            detKeypointsHarris(keypoints, imgGray, time, false);
        }
	else
	{
	    detKeypointsModern(keypoints, imgGray, detectorType, time, false);
	}
        //// EOF STUDENT ASSIGNMENT

	keypointImages.push_back(keypoints.size());
	timeImageKeypoints.push_back(time);

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            for(auto it = keypoints.begin(); it < keypoints.end(); it++){
		if (vehicleRect.x < it->pt.x && vehicleRect.x + vehicleRect.width > it->pt.x)
		{
		    if(vehicleRect.y < it->pt.y && vehicleRect.y + vehicleRect.height > it->pt.y)
		    {
			keypointsrect.push_back(*it);
		    }
		}
	    }
        }

	keypoints = keypointsrect;
	pvkeypointImages.push_back(keypoints.size());
	cout << "Keypoints on preceding vehicle : " << keypoints.size() << endl;
        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */
	string keypointImageName = dataPath + "imagesResults/" + (*it1) +"/"+(*it1)+"_"+to_string(imgIndex+1)+".jpeg";
	cv::Mat kptImg = ((dataBuffer.end() - 1)->cameraImg).clone();
	cv::drawKeypoints((dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints, kptImg, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
	cv::imwrite(keypointImageName, kptImg);

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
	string descriptorType = (*it2);
	
	descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType, time);
	timeImageDescriptors.push_back(time);
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
            string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
            //string selectorType = "SEL_NN";       // SEL_NN, SEL_KNN
	    if ((*it2).compare("SIFT") == 0 ){
		    descriptorType = "DES_HOG";
	    }
            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
	    //string matcherType = "MAT_FLANN";
	    string selectorType = "SEL_KNN";

            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType, matcherType, selectorType);

            //// EOF STUDENT ASSIGNMENT
	    matchesImage.push_back(matches.size());
            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;
	    cout << "Number of matches : " << matches.size() << endl;
            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
            bVis = true;
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

		string imgResultName = imgResultFolder + (*it1) + "/" + (*it2) + "/" +(*it1)+ "_" + (*it2) + "_" + to_string(imgPairCounter) +".jpeg";
		imgPairCounter++;
		cv::imwrite(imgResultName, matchImg);
                //string windowName = "Matching keypoints between two camera images";
                //cv::namedWindow(windowName, 7);
                //cv::imshow(windowName, matchImg);
                //cout << "Press key to continue to next image" << endl;
                //cv::waitKey(0); // wait for key to be pressed
            }
            bVis = false;
        }

    } // eof loop over all images
    }
    //detectorKeypoints.push_back(vector<int>{keypointImages.begin(), keypointImages.begin()+10});
    //pvdetectorKeypoints.push_back(vector<int>{pvkeypointImages.begin(), pvkeypointImages.begin()+10});
    //timeKeypoints.push_back(vector<double>{timeImageKeypoints.begin(), timeImageKeypoints.begin()+10});
    //timeDescriptors.push_back(timeImageDescriptors);
    //matchesKeypoints.push_back(matchesImage);
    file << "All Keypoints" + (*it1) <<","<<"Preceding Vehicle Keypoints"<<","<<"Detection Time"<<","<<"Extraction Time"<<","<<"# Matches"<< endl;
    for (size_t i = 0; i < keypointImages.size(); i++){   	
		file << to_string(keypointImages[i]) << "," << to_string(pvkeypointImages[i]) << "," << to_string(timeImageKeypoints[i]) << "," << to_string(timeImageDescriptors[i]) << "," << to_string(matchesImage[i]) <<endl;

    }
    }
    return 0;
}

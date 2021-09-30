
#include <iostream>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, int imgNum, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    //cv::namedWindow(windowName, 1);

    string filename = "../Results/TopViewImages/topview_" + to_string(imgNum) + ".jpeg";
    cv::imwrite(filename, topviewImg);
    //cv::imshow(windowName, topviewImg);

    /*if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }*/
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
  	vector<double> dist;
	for (auto it1 = boundingBox.kptMatches.begin(); it1 != boundingBox.kptMatches.end();){
		cv::KeyPoint kptoutercurr = kptsCurr[(*it1).trainIdx];
		cv::KeyPoint kptouterprev = kptsPrev[(*it1).queryIdx];	
		if(cv::norm(kptoutercurr.pt - kptouterprev.pt) > 10.0){
			boundingBox.kptMatches.erase(it1);
		}
		else{
			it1++;
		}
	}  
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
	vector<double> distRatios;
	for (auto it1 = kptMatches.begin(); it1 < kptMatches.end() - 1; ++it1){
		cv::KeyPoint kptoutercurr = kptsCurr[(*it1).trainIdx];
		cv::KeyPoint kptouterprev = kptsPrev[(*it1).queryIdx];
		for (auto it2 = kptMatches.begin() + 1; it2 < kptMatches.end(); ++it2){
			cv::KeyPoint kptinnercurr = kptsCurr[(*it2).trainIdx];
			cv::KeyPoint kptinnerprev = kptsPrev[(*it2).queryIdx];

			double distCurr = cv::norm(kptoutercurr.pt - kptinnercurr.pt);
			double distPrev = cv::norm(kptouterprev.pt - kptinnerprev.pt);

			if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= 100 && distCurr < 1000){

                		double distRatio = distCurr / distPrev;
               			distRatios.push_back(distRatio);
            		}

		}	
	
	}

	double meanDistRatio = accumulate(distRatios.begin(), distRatios.end(), 0.0)/distRatios.size();
	double medianDistRatio = 0;
	sort(distRatios.begin(), distRatios.end());
	if (distRatios.size() % 2 == 0){
        	medianDistRatio = 0.5*(distRatios[distRatios.size()/2] + distRatios[distRatios.size()/2+1]);
    	}
    	else
        	medianDistRatio = distRatios[(distRatios.size()+1)/2];

	double dT = 1 / frameRate; 	
	//TTC = - dT / (1 - meanDistRatio); 
	TTC = - dT / (1- medianDistRatio); 
}


bool lidarsort(const LidarPoint a, const LidarPoint b){
	return a.x < b.x;
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{   
    double minXPrev, minXCurr;
    sort(lidarPointsPrev.begin(), lidarPointsPrev.end(), lidarsort);
    sort(lidarPointsCurr.begin(), lidarPointsCurr.end(), lidarsort);

    if (lidarPointsPrev.size() % 2 == 0){
    	minXPrev = 0.5*(lidarPointsPrev[lidarPointsPrev.size()/2].x + lidarPointsPrev[lidarPointsPrev.size()/2 + 1].x);
    }
    else{
    	minXPrev = lidarPointsPrev[(lidarPointsPrev.size()+1)/2].x;
    }
    if (lidarPointsCurr.size() % 2 == 0){
    	minXCurr = 0.5*(lidarPointsCurr[lidarPointsCurr.size()/2].x + lidarPointsCurr[lidarPointsCurr.size()/2 + 1].x);
    }
    else{
    	minXCurr = lidarPointsCurr[(lidarPointsCurr.size()+1)/2].x;
    }
    
    TTC = minXCurr / ((minXPrev - minXCurr) * frameRate);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
	for (auto it1 = currFrame.boundingBoxes.begin(); it1 < currFrame.boundingBoxes.end(); it1++){
		for (auto it2 = currFrame.keypoints.begin(); it2 < currFrame.keypoints.end(); it2++){
			if ((*it1).roi.contains((*it2).pt)){
				(*it1).keypoints.push_back((*it2));	
			}
		}
		for (auto it3 = matches.begin(); it3 < matches.end(); it3++){
			if((*it1).roi.contains(currFrame.keypoints[(*it3).trainIdx].pt)){
				(*it1).kptMatches.push_back((*it3));
			}
		}
	}
	for (auto it1 = prevFrame.boundingBoxes.begin(); it1 < prevFrame.boundingBoxes.end(); it1++){
		for (auto it2 = prevFrame.keypoints.begin(); it2 < prevFrame.keypoints.end(); it2++){
			if ((*it1).roi.contains((*it2).pt)){
				(*it1).keypoints.push_back((*it2));	
			}
		}
		for (auto it3 = matches.begin(); it3 < matches.end(); it3++){
			if((*it1).roi.contains(prevFrame.keypoints[(*it3).queryIdx].pt)){
				(*it1).kptMatches.push_back((*it3));
			}
		}
	}
	/*for (auto it = matches.begin(); it < matches.begin()+10; it++){
		cout << "Query Idx " << (*it).queryIdx << " Train Idx " << (*it).trainIdx << endl;
		cout << " X1 "<< currFrame.keypoints[(*it).trainIdx].pt.x << " X2 " << prevFrame.keypoints[(*it).queryIdx].pt.x << endl;
	}*/

	for (auto it1 = currFrame.boundingBoxes.begin(); it1 < currFrame.boundingBoxes.end(); it1++){
		int counterArray [prevFrame.boundingBoxes.size()] = {};
		for (auto it2 = (*it1).kptMatches.begin(); it2 < (*it1).kptMatches.end(); it2++){
			for (int it3 = 0; it3 < prevFrame.boundingBoxes.size(); it3++){
				if (prevFrame.boundingBoxes[it3].roi.contains(prevFrame.keypoints[(*it2).queryIdx].pt)){
					counterArray[it3]++;
				}
			}
		}
		//cout << "# of prev bounding boxes " << prevFrame.boundingBoxes.size() << endl;
		int prevboxID = distance(counterArray, max_element(counterArray, counterArray + sizeof(counterArray)/sizeof(int)));
		bbBestMatches.insert(pair<int, int>((*it1).boxID, prevFrame.boundingBoxes[prevboxID].boxID));
	}
	//cout << bbBestMatches.size() << endl;
}

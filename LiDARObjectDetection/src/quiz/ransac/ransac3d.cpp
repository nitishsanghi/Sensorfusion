/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <iostream>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -10; i < 10; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
        double rz = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = i+scatter*rz;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 20;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
        double rz = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 5*rz;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	for (pcl::PointXYZ point : cloud->points){

		//std::cout << point.x << " : " << point.y << " : " << point.z << std::endl;

	}
	
	int size_cloud = cloud->points.size();
	
	// For max iterations 
	for (int i = 0; i < maxIterations; i++){
	// Randomly sample subset and fit line
		int point1 = 0, point2 = 0;
		point1 = rand() % size_cloud;
		point2 = rand() % size_cloud;
		while(point1 == point2){
			point2 = rand() % size_cloud;
		}
		double x0 = cloud->points[point1].x;
		double y0 = cloud->points[point1].y;
		double x1 = cloud->points[point2].x;
		double y1 = cloud->points[point2].y;
		double a = (y0 - y1);
		double b = (x1 - x0);
		double c = (x0*y1 - y0*x1);
		std::unordered_set<int> inliers_temp;
		for (int j = 0; j<size_cloud; j++){
			double d = abs(a*cloud->points[j].x + b*cloud->points[j].y + c)/sqrt(a*a + b*b);
			//std::cout << "value of d : " << d << std::endl;
			if (d < distanceTol) {
				inliers_temp.insert(j);
			}
		}
		if (inliersResult.size() < inliers_temp.size()){
				inliersResult = inliers_temp;
		}
		//std::cout << "Number of inliers : " << inliersResult.size() << std::endl;
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
	}
	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	for (pcl::PointXYZ point : cloud->points){

		//std::cout << point.x << " : " << point.y << " : " << point.z << std::endl;

	}
	
	int size_cloud = cloud->points.size();
	
	// For max iterations 
	for (int i = 0; i < maxIterations; i++){
	// Randomly sample subset and fit line
		int point1 = 0, point2 = 0, point3 = 0;
		point1 = rand() % size_cloud;
		point2 = rand() % size_cloud;
		point3 = rand() % size_cloud;
		while(point1 == point2 || point1 == point3 || point2 == point3){
			point2 = rand() % size_cloud;
			point3 = rand() % size_cloud;
		}
		//Point 1
		double x1 = cloud->points[point1].x;
		double y1 = cloud->points[point1].y;
		double z1 = cloud->points[point1].z;

		//Point 2
		double x2 = cloud->points[point2].x;
		double y2 = cloud->points[point2].y;
		double z2 = cloud->points[point2].z;

		//Point 3
		double x3 = cloud->points[point3].x;
		double y3 = cloud->points[point3].y;
		double z3 = cloud->points[point3].z;

		//Plane coefficients
		double A = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
		double B = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
		double C = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
		double D = -(A*x1 + B*y1 + C*z1);
		std::unordered_set<int> inliers_temp;
		for (int j = 0; j<size_cloud; j++){
			double d = abs(A*cloud->points[j].x + B*cloud->points[j].y + C*cloud->points[j].z + D)/sqrt(A*A + B*B + C*C);
			//std::cout << "value of d : " << d << std::endl;
			if (d < distanceTol) {
				inliers_temp.insert(j);
			}
		}
		if (inliersResult.size() < inliers_temp.size()){
				inliersResult = inliers_temp;
		}
		//std::cout << "Number of inliers : " << inliersResult.size() << std::endl;
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
	}
	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 500, 0.1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 3D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(1,1,1));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}

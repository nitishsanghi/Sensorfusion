// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"



//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr filterCloud (new pcl::PointCloud<PointT>);
    typename pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*filterCloud);

    typename pcl::PointCloud<PointT>::Ptr roiCloud (new pcl::PointCloud<PointT>);
    typename pcl::CropBox<PointT>::CropBox roi;
    roi.setMax(maxPoint);
    roi.setMin(minPoint);
    roi.setInputCloud(filterCloud);
    roi.filter(*roiCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return roiCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloud_obs (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_nobs (new pcl::PointCloud<PointT>);
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_nobs);

    extract.setNegative (true);
    extract.filter(*cloud_obs);
    

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_nobs, cloud_obs);
    return segResult;
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	int size_cloud = cloud->points.size();
	std::cout << "Cloud size : " << size_cloud << std::endl;
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
        //std::cout << point1 << " " << point2 << " " << point3 << std::endl;
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
				//std::cout << inliers_temp.size() << std::endl;
                inliersResult = inliers_temp;
                //std::cout << inliersResult.size() << std::endl;
		}
    }
	return inliersResult;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // TODO:: Fill in this function to find inliers for the cloud.
    std::unordered_set<int> inliers = RansacPlane(cloud, maxIterations, distanceThreshold);

    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		auto point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudInliers, cloudOutliers);
    return segResult;
}


/*template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold); 

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}*/


/*template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Segment(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
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
	
	//return inliersResult;


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}*/

template <typename PointT>
void ProcessPointClouds<PointT>::proximity(int i, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &cluster, std::vector<bool> &processed, KdTree<PointT>* tree, float distanceTol){
	processed[i] = true;
	cluster.push_back(i);
	std::vector<int> near_points = tree->search(cloud->points[i], distanceTol);
	for (auto j : near_points){
		if(!processed[j]){
			proximity(j, cloud, cluster, processed, tree, distanceTol);
		}
	}
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    //std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    //Creating KdTree for search
    //typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    KdTree<PointT>* tree = new KdTree<PointT>();
    
    for (int i=0; i<cloud->points.size(); i++){
        tree->insert(cloud->points[i],i);
    }

    std::vector<bool> processed(cloud->points.size(), false);

	std::vector<std::vector<int>> clusters;
	int i = 0;
	while (i < cloud->points.size()){
		if(processed[i]){
			i++; 
			continue;
		}
		std::vector<int> cluster;
		proximity(i, cloud, cluster, processed, tree, clusterTolerance);
		clusters.push_back(cluster);
		i++;

	}
 

    std::vector<typename pcl::PointCloud<PointT>::Ptr> cloudClusters;
    for (auto clusterId : clusters){
        typename pcl::PointCloud<PointT>::Ptr singleCluster(new pcl::PointCloud<PointT>());
        for(auto id : clusterId){
            singleCluster->points.push_back(cloud->points[id]);
        }
        if (singleCluster->size() >= minSize && singleCluster->size() <= maxSize)
            cloudClusters.push_back(singleCluster);
    }



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return cloudClusters;
}


/*template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    //Creating KdTree for search
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it= cluster_indices.begin(); it != cluster_indices.end(); ++it){
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (const auto& idx: it->indices){
            cloud_cluster->push_back ((*cloud)[idx]);
        }
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}*/

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
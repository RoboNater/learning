// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

bool bbPrintMyDebug = True; // File-scope behavior control: if True, print my debug and info

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

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
   ProcessPointClouds<PointT>::ReferencePCL_SegmentPlane_DoNotUse(
       typename pcl::PointCloud<PointT>::Ptr cloud, 
       int maxIterations, 
       float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.
    
    // Segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
} // ReferencePCL_SegmentPlane_DoNotUse

double distFromPlane(double A, double B, double C, double D, double x, double y, double z)
{
	// Plane formula:  Ax + By + Cz +D = 0
	// Point (x,y,z)
	// Distance d = abs(Ax + By + Cz + D) / sqrt(A^2 + B^2 + C^2)
	double t1 = std::abs(A*x + B*y + C*z + D);
	double t2 = std::sqrt(A*A + B*B + C*C);
	double d = t1 / t2;
	return d;
} // distFromPlane

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
	int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;

	srand(time(NULL));
	int lnNumPoints = cloud->points.size();
    if (bbPrintMyDebug) 
    {
	    cout << "We have " << lnNumPoints << " points." << std::endl;
	    cout << "Starting " << maxIterations << " iterations of RANSAC." << std::endl;
    }

	int lnBestNumInliers = 0;
	int liBestP1 = 0;
	int liBestP2 = 1;
    int liBestP3 = 2;

	// For max iterations 
	for (int liIt = 0; liIt < maxIterations; liIt++)
	{
		int p1, p2, p3;		
		// Randomly sample subset and fit line
		p1 = (int) (lnNumPoints * ((double) rand()) / (RAND_MAX));
		do 
		{
			p2 = (int) (lnNumPoints * ((double) rand()) / (RAND_MAX));
		} while (p2 == p1);
		do
		{
			p3 = (int) (lnNumPoints * ((double) rand()) / (RAND_MAX));
		} while ((p3 == p1) || (p3 == p2));
	

		pcl::PointXYZ point1 = cloud->points[p1];
		pcl::PointXYZ point2 = cloud->points[p2];
		pcl::PointXYZ point3 = cloud->points[p3];
        if (bbPrintMyDebug) 
        {
		    cout << "p1: " << p1 << "  Point[p1]: [ " << point1.x << ", " << point1.y << " ]" << std::endl;
        }

		double A = (point2.y-point1.y)*(point3.z-point1.z) - (point2.z-point1.z)*(point3.y-point1.y);
		double B = (point2.z-point1.z)*(point3.x-point1.x) - (point2.x-point1.x)*(point3.z-point1.z);
		double C = (point2.x-point1.x)*(point3.y-point1.y) - (point2.y-point1.y)*(point3.x-point1.x);
		double D = -(A*point1.x + B*point1.y + C*point1.z);

		int lnNumInliers = 0;
		double sumSquaredDistance = 0;
		double sumAbsDistance     = 0;
		// Measure distance between every point and fitted line
		for(int liP = 0; liP < cloud->points.size(); liP++)
		{
			pcl::PointXYZ point = cloud->points[liP];
			double d = distFromPlane(A,B,C,D,point.x,point.y,point.z);

			sumAbsDistance     += std::abs(d);
			sumSquaredDistance += d*d;
      		// If distance is smaller than threshold count it as inlier
			if (d < distanceTol)
			{
				lnNumInliers++;
			}

		}
		double l1Distance = sumAbsDistance / (double)lnNumPoints;
		double l2Distance = std::sqrt(sumSquaredDistance / (double)lnNumPoints);
		 
        if (lnNumInliers > lnBestNumInliers)
		{
			liBestP1 = p1;
			liBestP2 = p2;
			liBestP3 = p3;
			lnBestNumInliers = lnNumInliers;
		}
        if (bbPrintMyDebug) 
        {        
            cout << "It: " << liIt << "  l1: " << l1Distance << "  l2: " << l2Distance \
                << "  NumIn: " << lnNumInliers << "  BestIn: " << lnBestNumInliers << std::endl;
        }
	}
	// Return indicies of inliers from fitted line with most inliers
	int p1 = liBestP1;
	int p2 = liBestP2;
	int p3 = liBestP3;
	pcl::PointXYZ point1 = cloud->points[p1];
	pcl::PointXYZ point2 = cloud->points[p2];
	pcl::PointXYZ point3 = cloud->points[p3];
	double A = (point2.y-point1.y)*(point3.z-point1.z) - (point2.z-point1.z)*(point3.y-point1.y);
	double B = (point2.z-point1.z)*(point3.x-point1.x) - (point2.x-point1.x)*(point3.z-point1.z);
	double C = (point2.x-point1.x)*(point3.y-point1.y) - (point2.y-point1.y)*(point3.x-point1.x);
	double D = -(A*point1.x + B*point1.y + C*point1.z);
	// Measure distance between every point and fitted plane
	for(int liP = 0; liP < cloud->points.size(); liP++)
	{
		pcl::PointXYZ point = cloud->points[liP];
		double d = distFromPlane(A,B,C,D,point.x,point.y,point.z);
		// If distance is smaller than threshold count it as inlier
		if (d < distanceTol)
		{
			inliersResult.insert(liP);
		}

	}

	return inliersResult;

} // RansacPlane

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
   ProcessPointClouds<PointT>::SegmentPlane(
       typename pcl::PointCloud<PointT>::Ptr cloud, 
       int maxIterations, 
       float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.
    
    // Segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, 
    float clusterTolerance, 
    int minSize, 
    int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


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
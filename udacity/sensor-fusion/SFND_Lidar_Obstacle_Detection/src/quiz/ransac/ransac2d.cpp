/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <iostream>
#include <cmath>
pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

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

double distFromLine(double A, double B, double C, double x, double y)
{
	// Line formula:  Ax + By + C = 0
	// Point (x,y)
	// Distance d = abs(Ax + By + C) / sqrt(A^2 + B^2)
	double t1 = std::abs(A*x + B*y + C);
	double t2 = std::sqrt(A*A + B*B);
	double d = t1 / t2;
	return d;
}


double distFromPlane(double A, double B, double C, double D, double x, double y, double z)
{
	// Plane formula:  Ax + By + Cz +D = 0
	// Point (x,y,z)
	// Distance d = abs(Ax + By + Cz + D) / sqrt(A^2 + B^2 + C^2)
	double t1 = std::abs(A*x + B*y + C*z + D);
	double t2 = std::sqrt(A*A + B*B + C*C);
	double d = t1 / t2;
	return d;
}


std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
	int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	int lnNumPoints = cloud->points.size();
	// cout << "We have " << lnNumPoints << " points." << std::endl;
	// cout << "Starting " << maxIterations << " iterations of RANSAC." << std::endl;

	// TODO: Fill in this function

	int lnBestNumInliers = 0;
	int liBestP1 = 0;
	int liBestP2 = 1;


	// For max iterations 
	for (int liIt = 0; liIt < maxIterations; liIt++)
	{
		
		// Randomly sample subset and fit line
		int p1 = (int) (lnNumPoints * ((double) rand()) / (RAND_MAX));
		int p2 = p1;
		while (p2 == p1)
		{ p2 = (int) (lnNumPoints * ((double) rand()) / (RAND_MAX));}

		// cout << "p1 = " << p1 << ", p2 = " << p2 << std::endl;
		// double m = cloud->points[index]
		pcl::PointXYZ point1 = cloud->points[p1];
		pcl::PointXYZ point2 = cloud->points[p2];
		// cout << "p1: " << p1 << "  Point[p1]: [ " << point1.x << ", " << point1.y << " ]" << std::endl;
  		// point.x = i+scatter*rx;
		// Ax + By + C = 0
		// (y1 - y2)x + (x2 - x1)y + (x1*y2 - x2*y1) = 0
		double A = point1.y - point2.y;
		double B = point1.x - point2.x;
		double C = point1.x*point2.y - point2.x*point1.y;

		int lnNumInliers = 0;
		double sumSquaredDistance = 0;
		double sumAbsDistance     = 0;
		// Measure distance between every point and fitted line
		for(int liP = 0; liP < cloud->points.size(); liP++)
		{
			pcl::PointXYZ point = cloud->points[liP];
			double d = distFromLine(A,B,C,point.x,point.y);
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
			lnBestNumInliers = lnNumInliers;
		}
		// cout << "It: " << liIt << "  l1: " << l1Distance << "  l2: " << l2Distance \
		//     << "  NumIn: " << lnNumInliers << "  BestIn: " << lnBestNumInliers << std::endl;
	}
	// Return indicies of inliers from fitted line with most inliers
	int p1 = liBestP1;
	int p2 = liBestP2;
	pcl::PointXYZ point1 = cloud->points[p1];
	pcl::PointXYZ point2 = cloud->points[p2];
	double A = point1.y - point2.y;
	double B = point1.x - point2.x;
	double C = point1.x*point2.y - point2.x*point1.y;
	// Measure distance between every point and fitted line
	for(int liP = 0; liP < cloud->points.size(); liP++)
	{
		pcl::PointXYZ point = cloud->points[liP];
		double d = distFromLine(A,B,C,point.x,point.y);
		// If distance is smaller than threshold count it as inlier
		if (d < distanceTol)
		{
			inliersResult.insert(liP);
		}

	}

	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
	int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;

	srand(time(NULL));
	int lnNumPoints = cloud->points.size();
	// cout << "We have " << lnNumPoints << " points." << std::endl;
	// cout << "Starting " << maxIterations << " iterations of RANSAC." << std::endl;

	// TODO: Fill in this function

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
	

		// cout << "p1 = " << p1 << ", p2 = " << p2 << std::endl;
		// double m = cloud->points[index]
		pcl::PointXYZ point1 = cloud->points[p1];
		pcl::PointXYZ point2 = cloud->points[p2];
		pcl::PointXYZ point3 = cloud->points[p3];
		// cout << "p1: " << p1 << "  Point[p1]: [ " << point1.x << ", " << point1.y << " ]" << std::endl;

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
		cout << "It: " << liIt << "  l1: " << l1Distance << "  l2: " << l2Distance \
		     << "  NumIn: " << lnNumInliers << "  BestIn: " << lnBestNumInliers << std::endl;
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

}



int main ()
{
    bool cbLineOnly = false;

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	
	std::unordered_set<int> inliers;
	if (cbLineOnly == true)
	{
	    inliers = Ransac(cloud, 100, 0.3);
	}
	else
	{
	    inliers = RansacPlane(cloud, 100, 0.3);
	}
	

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


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
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

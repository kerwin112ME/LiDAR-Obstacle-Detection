/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <random>
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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::vector<int> indices;
	int n = cloud->size();
	srand(time(NULL));
	cout << maxIterations<<endl;
	while(maxIterations --) {
		std::unordered_set<int> inliers;
		while(inliers.size()<2) inliers.insert(rand() % cloud->points.size());
	
		double x1, x2, y1, y2;
		auto iter = inliers.begin();
		x1 = cloud->points[*iter].x;
		y1 = cloud->points[*iter].y;
		iter ++;
		x2 = cloud->points[*iter].x;
		y2 = cloud->points[*iter].y;

		double A, B, C;
		A = y1 - y2;
		B = x2 - x1;
		C = (x1*y2 - x2*y1);

		for(int i=0; i<cloud->points.size(); i++) {
			if(inliers.find(i) != inliers.end()) continue;
			pcl::PointXYZ pt = cloud->points[i];
			double x3 = pt.x, y3 = pt.y;
			double d = fabs(A*x3+B*y3+C)/sqrt(A*A + B*B);
			if(d <= distanceTol)
				inliers.insert(i);
		}

		if(inliers.size() > inliersResult.size()) inliersResult = inliers;
	}
	
	return inliersResult;

}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::vector<int> indices;
	int n = cloud->size();
	srand(time(NULL));
	
	while(maxIterations --) {
		std::unordered_set<int> inliers;
		while(inliers.size()<3) inliers.insert(rand() % cloud->points.size());
	
		float x1, x2, x3, y1, y2, y3, z1, z2, z3;
		auto iter = inliers.begin();
		x1 = cloud->points[*iter].x;
		y1 = cloud->points[*iter].y;
		z1 = cloud->points[*iter].z;
		iter ++;
		x2 = cloud->points[*iter].x;
		y2 = cloud->points[*iter].y;
		z2 = cloud->points[*iter].z;
		iter ++;
		x3 = cloud->points[*iter].x;
		y3 = cloud->points[*iter].y;
		z3 = cloud->points[*iter].z;

		float ni, nj, nk;
		ni = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
		nj = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
		nk = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);

		float A = ni, B = nj, C = nk, D = -(ni*x1 + nj*y1 + nk*z1);

		for(int i=0; i<cloud->points.size(); i++) {
			if(inliers.find(i) != inliers.end()) continue;
			pcl::PointXYZ pt = cloud->points[i];
			float x = pt.x, y = pt.y, z = pt.z;
			float d = fabs(A*x+B*y+C*z+D) / sqrt(A*A + B*B + C*C);
			if(d <= distanceTol)
				inliers.insert(i);
		}

		if(inliers.size() > inliersResult.size()) inliersResult = inliers;
	}
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3D(cloud, 100, 0.18);

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

/* \author Aaron Brown */
#ifndef RANSAC_H
#define RANSAC_H

#include <unordered_set>
#include <random>
#include <iostream>
#include <cmath>

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
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
			pcl::PointXYZI pt = cloud->points[i];
			double x3 = pt.x, y3 = pt.y;
			double d = fabs(A*x3+B*y3+C)/sqrt(A*A + B*B);
			if(d <= distanceTol)
				inliers.insert(i);
		}

		if(inliers.size() > inliersResult.size()) inliersResult = inliers;
	}
	
	return inliersResult;

}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
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
			pcl::PointXYZI pt = cloud->points[i];
			float x = pt.x, y = pt.y, z = pt.z;
			float d = fabs(A*x+B*y+C*z+D) / sqrt(A*A + B*B + C*C);
			if(d <= distanceTol)
				inliers.insert(i);
		}

		if(inliers.size() > inliersResult.size()) inliersResult = inliers;
	}
	
	return inliersResult;

}

#endif /* RANSAC_H */
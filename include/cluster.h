#ifndef CLUSTER_H
#define CLUSTER_H
#include "kdtree.h"
#include <climits>


void proximity(const std::vector<std::vector<float>>& points, const int &id, 
	KdTree *tree, std::vector<int> &cluster, bool isProcessed[], const float &distanceTol)
{
	isProcessed[id] = true;
	cluster.push_back(id);
	std::vector<int> nearby = tree->search(points[id], distanceTol);
	for(auto &nearbyId: nearby) 
		if(!isProcessed[nearbyId]) proximity(points, nearbyId, tree, cluster, isProcessed, distanceTol);
	
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize = 0, int maxSize = INT_MAX)
{
	bool isProcessed[points.size()] = {false};
	std::vector<std::vector<int>> clusters;

	for(int i=0; i<points.size(); i++) {
		if(!isProcessed[i]) {
			std::vector<int> cluster;
			proximity(points, i, tree, cluster, isProcessed, distanceTol);
      if(cluster.size() >= minSize && cluster.size() <= maxSize)
			 clusters.push_back(cluster);
		}
	}
 
	return clusters;

}

#endif /* CLUSTER_H */


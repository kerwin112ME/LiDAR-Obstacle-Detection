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
void ProcessPointClouds<PointT>::proximity(const std::vector<std::vector<float>>& points, const int &id, 
    KdTree *tree, std::vector<int> &cluster, bool isProcessed[], const float &distanceTol)
{
    isProcessed[id] = true;
    cluster.push_back(id);
    std::vector<int> nearby = tree->search(points[id], distanceTol);
    for(auto &nearbyId: nearby) 
        if(!isProcessed[nearbyId]) proximity(points, nearbyId, tree, cluster, isProcessed, distanceTol);
    
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize)
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

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac3D(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
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



template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, 
                                                        float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr cloud_vg(new pcl::PointCloud<PointT>());
    typename pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloud_vg);

    typename pcl::PointCloud<PointT>::Ptr cloud_ROI(new pcl::PointCloud<PointT>());
    typename pcl::CropBox<PointT> ROI(true);
    ROI.setInputCloud(cloud_vg);
    ROI.setMin(minPoint);
    ROI.setMax(maxPoint);
    ROI.filter(*cloud_ROI);

    std::vector<int> cropIndices;
    typename pcl::CropBox<PointT> deRoof(true);
    deRoof.setInputCloud(cloud_ROI);
    deRoof.setMin(Eigen::Vector4f(-2, -2, -2, 1));
    deRoof.setMax(Eigen::Vector4f(3, 2, 1, 1));
    deRoof.filter(cropIndices);

    pcl::PointIndices::Ptr roof(new pcl::PointIndices);
    for(int ind: cropIndices) roof->indices.push_back(ind);

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
    typename pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_ROI);
    extract.setIndices(roof);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    typename pcl::PointCloud<PointT>::Ptr obsCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

    for(int index: inliers->indices) planeCloud->points.push_back(cloud->points[index]);

    typename pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obsCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(planeCloud, obsCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    
    pcl::ModelCoefficients:: Ptr coefficients(new pcl::ModelCoefficients());

    typename pcl::SACSegmentation<PointT> plantSeg;
    plantSeg.setOptimizeCoefficients(true);
    plantSeg.setModelType(pcl::SACMODEL_PLANE);
    plantSeg.setMethodType(pcl::SAC_RANSAC);
    plantSeg.setMaxIterations(maxIterations);
    plantSeg.setDistanceThreshold(distanceThreshold);

    plantSeg.setInputCloud(cloud);
    plantSeg.segment(*inliers, *coefficients);

    if(inliers->indices.size() == 0) {
        PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, 
                                                                                    float clusterTolerance, int minSize, int maxSize)
{
    // output: vector<pointCloud<PointT>>
    // input: PointCloud<PointT> cloud, float clusterTolerance, int minSize, int maxSize
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for(auto it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>());
        for(auto pit = it->indices.begin(); pit != it->indices.end(); pit++) 
            cloud_cluster->push_back((*cloud)[*pit]);
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }

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

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
                ProcessPointClouds<PointT>::MySeparateClouds(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> planeIndices = Ransac3D(cloud, maxIterations, distanceTol); // 100, 0.2
    pcl::PointIndices::Ptr groundPlane (new pcl::PointIndices);
    for(auto &ind: planeIndices) groundPlane->indices.push_back(ind);

    typename pcl::PointCloud<PointT>::Ptr groundPlaneCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstacles (new pcl::PointCloud<PointT>());
    typename pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(groundPlane);
    extract.setNegative(false); // extract the inliers (ground plane)
    extract.filter(*groundPlaneCloud);
    extract.setNegative(true); // extract the obstacles
    extract.filter(*obstacles);

    return {groundPlaneCloud, obstacles};
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> 
            ProcessPointClouds<PointT>::MyClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // build the kd-tree
    KdTree *obstaclesTree (new KdTree());
    std::vector<std::vector<float>> obsPoints;
    for(int ind = 0; ind < cloud->points.size(); ind++) {
        auto data = cloud->points[ind];
        std::vector<float> point = {data.x, data.y, data.z};
        obsPoints.push_back(point);
        obstaclesTree->insert(point, ind);
    }

    std::vector<typename pcl::PointCloud<PointT>::Ptr> cloudClusters;
    std::vector<std::vector<int>> obsClusters = euclideanCluster(obsPoints, obstaclesTree, clusterTolerance, minSize, maxSize); // 0.47, 10, 2000
    for(std::vector<int> clusterIndices: obsClusters) {
        typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>());
        for(int &ind: clusterIndices)
            cluster->points.emplace_back(std::move(cloud->points[ind]));

        cloudClusters.push_back(cluster);
    }

    return cloudClusters;
}
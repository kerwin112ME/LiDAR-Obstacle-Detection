/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include <unordered_set>
#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
#include "processPointClouds.cpp" // using templates for processPointClouds so also include .cpp to help linker
#include "cluster.h"
#include "ransac.h"
#include "kdtree.h"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    Lidar *lidar = new Lidar(cars, 0.0);
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();

    //renderPointCloud(viewer, cloud, "inputCloud");

    ProcessPointClouds<pcl::PointXYZ> processor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = processor.SegmentPlane(cloud, 100, 0.2);
    //renderPointCloud(viewer, segmentCloud.first, "plane", Color(0,1,0));
    //renderPointCloud(viewer, segmentCloud.second, "obstacle", Color(1,0,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = processor.Clustering(segmentCloud.second, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster: cloudClusters) {
        std::cout << "cluster size ";
        processor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId]);
        ++ clusterId;

        Box box = processor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
    }

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, 
															const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  	///// downsize the cloud to increase effeciency /////
  	pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = 
  		pointProcessorI->FilterCloud(inputCloud, 0.2f, Eigen::Vector4f(-20, -6, -2, 1), Eigen::Vector4f(20, 7, 2, 1));

  	///// segment the ground plane from other obstacles /////
  	std::unordered_set<int> planeIndices = Ransac3D(filterCloud, 100, 0.2);
  	pcl::PointIndices::Ptr groundPlane (new pcl::PointIndices);
  	for(auto &ind: planeIndices) groundPlane->indices.push_back(ind);

  	pcl::PointCloud<pcl::PointXYZI>::Ptr groundPlaneCloud (new pcl::PointCloud<pcl::PointXYZI>());
  	pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles (new pcl::PointCloud<pcl::PointXYZI>());
  	pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(filterCloud);
    extract.setIndices(groundPlane);
    extract.setNegative(false); // extract the inliers (ground plane)
    extract.filter(*groundPlaneCloud);
    extract.setNegative(true); // extract the obstacles
    extract.filter(*obstacles);

    renderPointCloud(viewer, groundPlaneCloud, "plane", Color(0,1,0));

    ///// build the kd-tree /////
    KdTree *obstaclesTree (new KdTree());
    std::vector<std::vector<float>> obsPoints;
    for(int ind = 0; ind < obstacles->points.size(); ind++) {
    	auto data = obstacles->points[ind];
    	std::vector<float> point = {data.x, data.y, data.z};
    	obsPoints.push_back(point);
    	obstaclesTree->insert(point, ind);
    }

  	///// clustering the obstacles /////
  	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters;
    std::vector<std::vector<int>> obsClusters = euclideanCluster(obsPoints, obstaclesTree, 0.5, 10, 2000);
    for(std::vector<int> clusterIndices: obsClusters) {
    	pcl::PointCloud<pcl::PointXYZI>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZI>());
    	for(int &ind: clusterIndices)
    		cluster->points.emplace_back(std::move(obstacles->points[ind]));

    	cloudClusters.push_back(cluster);
    }

  	int clusterId = 0;
  	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

  	///// bounding box /////
  	for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster: cloudClusters) {
      	//std::cout << "cluster size ";
      	//pointProcessorI->numPoints(cluster);
      	renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId]);
      	++ clusterId;

      	Box box = pointProcessorI->BoundingBox(cluster);
      	renderBox(viewer, box, clusterId);
  	}
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = TopDown;
    initCamera(setAngle, viewer);
    
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI (new pcl::PointCloud<pcl::PointXYZI>());

    while (!viewer->wasStopped ())
    {	
    	// Clear viewer
    	viewer->removeAllPointClouds();
    	viewer->removeAllShapes();

    	// Load pcd and run obstacle detection ppocess
    	inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    	cityBlock(viewer, pointProcessorI, inputCloudI);

    	streamIterator ++;
    	if(streamIterator == stream.end()) streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}
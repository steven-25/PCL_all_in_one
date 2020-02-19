/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <pcl/console/parse.h>

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


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>*& pointProcessorI, pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloudI)
{
    
    Eigen::Vector4f min(-5.0, -5.0, -5.0,1.0);
    Eigen::Vector4f max(5.0, 5.0, 5.0,1.0);
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputFilteredCloud = pointProcessorI->FilterCloud(inputCloudI, 0.1f, min, max);
    std::pair< pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segment = pointProcessorI->SegmentPlane(inputCloudI, 100, 0.2);
    renderPointCloud(viewer, segment.first, "plancloud", Color(1,0,0));       //输出点云分割地面的结果
    renderPointCloud(viewer, segment.second, "obstcloud", Color(0,1,0));      //输出点云分割障碍物的结果

   //障碍物点云聚类，然后给予bouning box的结果。
    
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudcluster = pointProcessorI->Clustering(segment.second, 1.0, 3, 30);
    std::cout<<"num of cluster："<<cloudcluster.size()<<endl;
    std::vector<Color> colors = {Color(1,0,0),Color(0,1,0),Color(0,0,1)};
    int clusterid = 0;
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster: cloudcluster){
        std::cout<<"size of cluster"+std::to_string(clusterid)<<":";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obcluster"+std::to_string(clusterid), colors[clusterid % colors.size()]);
        
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterid);
        clusterid++;
    }
	
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
   
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud = lidar->scan();
    //renderPointCloud(viewer,inputcloud,"pointoflidar");

     
     // ----------------------------------------------------
    // ------------------Input real data -------------------
    // ----------------------------------------------------
    
    
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointprocessor;
    std::pair< pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segment = pointprocessor.SegmentPlane(inputcloud, 100, 0.2);
    renderPointCloud(viewer, segment.first, "plancloud", Color(1,0,0));       //输出点云分割地面的结果
    renderPointCloud(viewer, segment.second, "obstcloud", Color(0,1,0));      //输出点云分割障碍物的结果

    //61-73是障碍物点云聚类，然后给予bouning box的结果。
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudcluster = pointprocessor.Clustering(segment.second, 1.0, 3, 30);
    std::cout<<"num of cluster："<<cloudcluster.size()<<endl;
    std::vector<Color> colors = {Color(1,0,0),Color(0,1,0),Color(0,0,1)};
    int clusterid = 0;
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster: cloudcluster){
        std::cout<<"size of cluster"+std::to_string(clusterid)<<":";
        pointprocessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obcluster"+std::to_string(clusterid), colors[clusterid % colors.size()]);
        
        Box box = pointprocessor.BoundingBox(cluster);
        renderBox(viewer,box,clusterid);
        clusterid++;
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


int main (int argc, char** argv)
{
    if (pcl::console::find_switch(argc, argv, "-h"))
  {
	std::cout << "usage:" << argv[0] << "\n"
		  << "[-h help]" << "\n"
		  << "[-p path/to/folder/pcd]" << "\n"
		  << std::endl;
	return 0;
  }

    std::string Path;
    std::string OutputPath;
    pcl::console::parse_argument(argc, argv, "-p", Path);

    std::cout << "starting enviroment Path:" << Path << std::endl;


    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);    



    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd(Path);
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    

    while (!viewer->wasStopped ())
    {
	//clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
	
	//load pcd and run obstacle detection process
	inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
	cityBlock(viewer, pointProcessorI, inputCloudI);
	streamIterator++;
	if(streamIterator == stream.end())
	    streamIterator =stream.begin();
	
        viewer->spinOnce ();
    } 
    return 0;
}

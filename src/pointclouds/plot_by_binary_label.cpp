#include <iostream>
#include <filesystem>
#include <thread>

#include "arvc_utils.cpp"

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>

// Type Definitions ////////////////////////////////////////////////////////////
typedef pcl::PointXYZLNormal PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace fs = std::filesystem;


void plotCloud(PointCloud::Ptr &cloud)
{

  PointCloud::Ptr truss_cloud (new PointCloud);
  PointCloud::Ptr env_cloud (new PointCloud);

  // Create the filtering object
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("label");
  pass.setFilterLimits (0.0, 0.0);
  pass.setNegative(false);
  pass.filter (*env_cloud);
  pass.setNegative(true);
  pass.filter(*truss_cloud);

  pcl::visualization::PCLVisualizer visualizer;
  pcl::visualization::PointCloudColorHandlerCustom<PointT> truss_color (truss_cloud, 0,255,0);
  visualizer.addPointCloud<PointT>(truss_cloud, truss_color, "truss_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> env_color (env_cloud, 200,200,200);
  visualizer.addPointCloud<PointT>(env_cloud, env_color, "env_cloud");

  visualizer.addCoordinateSystem(0.01, "sensor_origin");


  while (!visualizer.wasStopped())
    visualizer.spinOnce(100);

}


int main(int argc, char **argv)
{
  fs::path current_path = fs::current_path();
  PointCloud::Ptr cloud_in (new PointCloud);

  if(argc < 2)
  {
    for(const auto &entry : fs::directory_iterator(current_path))
    {
    }
  }


  else
  {
    fs::path entry = argv[1];
    cloud_in = arvc::readCloud(entry);
    plotCloud(cloud_in);
  }

  return 0;  
}



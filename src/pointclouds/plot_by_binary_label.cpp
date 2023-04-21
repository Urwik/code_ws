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

void 
writeCloud(PointCloud::Ptr &cloud_in)
{
  pcl::PLYWriter ply_writer;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*cloud_in, *cloud_out);

  fs::path abs_file_path = fs::current_path().parent_path() / "FILTERED_CLOUD";
  if (!fs::exists(abs_file_path)) 
    fs::create_directory(abs_file_path);

  std::string filename = "nube.ply";

  abs_file_path = abs_file_path / filename;
  ply_writer.write(abs_file_path.string(), *cloud_out, true, false);
}

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

  // writeCloud(truss_cloud);

  pcl::visualization::PCLVisualizer visualizer;
  pcl::visualization::PointCloudColorHandlerCustom<PointT> truss_color (truss_cloud, 0,255,0);
  visualizer.addPointCloud<PointT>(truss_cloud, truss_color, "truss_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> env_color (env_cloud, 200,200,200);
  visualizer.addPointCloud<PointT>(env_cloud, env_color, "env_cloud");

  // visualizer.addCoordinateSystem(0.3, "sensor_origin");


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



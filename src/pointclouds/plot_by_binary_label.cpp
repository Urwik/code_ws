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


<<<<<<< HEAD
=======
myRGB randRGB()
{
  myRGB color;
  color.r = rand() % 256;
  color.g = rand() % 256;
  color.b = rand() % 256;

  return color;
}


void plotCloud(fs::path path_to_cloud, int min_points = 20, int max_points = 1000)
{
  PointCloud::Ptr cloud (new PointCloud);
  PointCloud::Ptr tmp_cloud (new PointCloud);
  PointCloud::Ptr truss_cloud (new PointCloud);


  pcl::PointIndices::Ptr indices (new pcl::PointIndices ());
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);

  std::string ext = path_to_cloud.extension();
  if(ext == ".pcd")
  {
    pcl::PCDReader reader;
    reader.read(path_to_cloud, *cloud);
  }  
  else
  {
    pcl::PLYReader reader;
    reader.read(path_to_cloud, *cloud);
  }
    
  std::cout << path_to_cloud.stem().string() << std::endl;
    
  indices.reset(new pcl::PointIndices);
  indices = findValue(cloud, 0);
  // std::cout << "Index " << i << ": " << indices->indices.size() << std::endl;

  extract.setIndices(indices);
  extract.filter(*tmp_cloud);

  extract.setNegative(true);
  extract.filter(*truss_cloud);

  pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color (tmp_cloud, 100, 100, 100);
  visualizer->addPointCloud<PointT>(tmp_cloud, single_color, "tmp_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> truss_color (truss_cloud, 0, 255, 0);
  visualizer->addPointCloud<PointT>(truss_cloud, truss_color, "truss_cloud");
}

void getlastkey()
{
  while(!visualizer->wasStopped())
  {
    std::getchar();
    button = true;
  }
}
>>>>>>> 9189f7c (commit caca)

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



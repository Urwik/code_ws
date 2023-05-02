#include <iostream>
#include <filesystem>
#include <thread>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>

// Type Definitions ////////////////////////////////////////////////////////////
typedef pcl::PointXYZL PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::visualization::PCLVisualizer pclVis;

namespace fs = std::filesystem;

pclVis::Ptr visualizer (new pclVis("Visualizer"));

bool button = false;

struct myRGB{
  int r;
  int g;
  int b;
};


pcl::PointIndices::Ptr findValue(PointCloud::Ptr cloud, const int value)
{
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  int index=0;
  for (auto point : *cloud)
  {
    if(point.label == value)
    {
      inliers->indices.push_back(index);
    }
    index++;
  }
  return inliers;
}


myRGB randRGB()
{
  myRGB color;
  color.r = rand() % 256;
  color.g = rand() % 256;
  color.b = rand() % 256;

  return color;
}


void plotCloud(fs::path path_to_cloud, int cloud_nmbr, int min_points = 20, int max_points = 1000, int viewport = 0)
{
  PointCloud::Ptr cloud (new PointCloud);
  PointCloud::Ptr tmp_cloud (new PointCloud);

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
    
  std::stringstream ss;
  
  for (size_t i = 0; i < 2; i++)
  {
    ss.str("");
    ss << "cloud_" << i << cloud_nmbr;
    indices.reset(new pcl::PointIndices);
    indices = findValue(cloud, i);
    // std::cout << "Index " << i << ": " << indices->indices.size() << std::endl;

    extract.setIndices(indices);
    extract.filter(*tmp_cloud);
    myRGB color;

    if (i==0)
    {
      color.r = 100;
      color.g = 100;
      color.b = 100;
    }
    else if (i==1) 
    {
      color.r = 0;
      color.g = 255;
      color.b = 0;
    }

    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color (tmp_cloud, color.r, color.g, color.b);
    visualizer->addPointCloud<PointT>(tmp_cloud, single_color, ss.str().c_str(), viewport);
  }
}


int main(int argc, char **argv)
{
  ////////////////////////////////////////////////////////////////////////////
  // VISUALIZATION
  pcl::visualization::PCLVisualizer vis ("PCL Visualizer");
  

  // Define cloud paths
  fs::path cloud_1("/home/fran/Desktop/ground_truth/00009.ply");
  fs::path cloud_2("/home/fran/Desktop/pointnet/00009.ply");
  fs::path cloud_3("/home/fran/Desktop/pointnet2/00009.ply");
  fs::path cloud_4("/home/fran/Desktop/minkowski/00009.ply");

  // Define ViewPorts
  int v1(0);
  int v2(0);
  int v3(0);
  int v4(0);

  // 2 HORIZONTAL VIEWPORTS
  // vis.createViewPort(0,0,0.5,1, v1);
  // vis.createViewPort(0.5,0,1,1, v2);

  // 3 HORIZONTAL VIEWPORTS
  // vis.createViewPort(0,0,0.33,1, v1);
  // vis.createViewPort(0.33,0,0.66,1, v2);
  // vis.createViewPort(0.66,0,1,1, v3);

  // 4 DISTRIBUTED VIEWPORTS
  visualizer->createViewPort(0,0.5,0.5,1, v1);
  visualizer->createViewPort(0.5,0.5,1,1, v2);
  visualizer->createViewPort(0,0,0.5,0.5, v3);
  visualizer->createViewPort(0.5,0,1,0.5, v4);

  plotCloud(cloud_1, 1, 1, 100000, v1);
  plotCloud(cloud_2, 2, 1, 100000, v2);
  plotCloud(cloud_3, 3, 1, 100000, v3);
  plotCloud(cloud_4, 4, 1, 100000, v4);



  while(!visualizer->wasStopped())
    visualizer->spinOnce(100);

  return 0;  
}



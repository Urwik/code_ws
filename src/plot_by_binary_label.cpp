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
typedef pcl::PointXYZI PointT;
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
    if(point.intensity == value)
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


void plotCloud(fs::path path_to_cloud, int min_points = 20, int max_points = 1000)
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
    
    
  std::stringstream ss;
  
  for (size_t i = 0; i < 2; i++)
  {
    ss.str("");
    ss << "cloud_" << i;
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
    visualizer->addPointCloud<PointT>(tmp_cloud, single_color, ss.str().c_str());
  }
}

void getlastkey()
{
  while(!visualizer->wasStopped())
  {
    std::getchar();
    button = true;
  }
}

int main(int argc, char **argv)
{
  // Get handlres for source and target cloud data /////////////////////////////
  fs::path current_path = fs::current_path();
  pcl::PCDReader cloud_reader;
  PointCloud::Ptr cloud (new PointCloud);

  std::thread check_keys(getlastkey);

  if(argc < 2)
  {
    for(const auto &entry : fs::directory_iterator(current_path))
    {
      plotCloud(entry.path(), 10, 10000000);
      while (!visualizer->wasStopped())
      {
        visualizer->spinOnce(100);
        if(button)
        {
          visualizer->removeAllPointClouds();
          button = false;
          break;
        }
      }
    }
  }

  else
  {
    std::string filename = argv[1];
    plotCloud(filename, 10, 10000000);
    while (!visualizer->wasStopped())
      visualizer->spinOnce(100);
  }

  return 0;  
}



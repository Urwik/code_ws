// cpp
#include <iostream>
#include <algorithm>
#include <filesystem>


// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>


//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;
pcl::visualization::PCLVisualizer::Ptr pclVisualizer (new pcl::visualization::PCLVisualizer ("PCL Visualizer"));

// ************************************************************************** //
namespace fs = boost::filesystem;

////////////////////////////////////////////////////////////////////////////////


int main(int argc, char **argv)
{
  // Get handlres for source and target cloud data /////////////////////////////
  fs::path current_path = fs::current_path();
  pcl::PCDReader cloud_reader;
  PointCloud::Ptr cloud (new PointCloud);
  bool button = false;


  if(argc < 2)
  {
    for(const auto &entry : fs::directory_iterator(current_path))
    {
      cloud_reader.read(entry.path().string(), *cloud);
      pclVisualizer->addPointCloud<PointT>(cloud, "cloud");
      
      button = false;
      while(!button)
      {
        if(std::getchar() == '\n')
          button = true;
        else
          pclVisualizer->spinOnce(100);
      }
      
      pclVisualizer->removeAllPointClouds();

    }
  }
  else
  {
    std::string filename = argv[1];
    // fs::path abs_file_path;
    // abs_file_path = current_path.c_str() + '/' + filename;

    cloud_reader.read(filename, *cloud);
    pclVisualizer->addPointCloud<PointT> (cloud, "cloud");

    while (!pclVisualizer->wasStopped())
      pclVisualizer->spinOnce(100);
  }

  return 0;
}

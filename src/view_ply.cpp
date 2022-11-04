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

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
pcl::visualization::PCLVisualizer::Ptr pclVisualizer (new pcl::visualization::PCLVisualizer ("PCL Visualizer"));

// ************************************************************************** //
namespace fs = boost::filesystem;

////////////////////////////////////////////////////////////////////////////////


int main(int argc, char **argv)
{
  // Get handlres for source and target cloud data /////////////////////////////
  fs::path current_path = fs::current_path();
  pcl::PLYReader ply_reader;

  if (argc <= 1)
    std::cout << "NEED TO SPECIFY THE FILE" << std::endl;
  
  std::string filename = argv[1];
  
  PointCloud::Ptr cloud (new PointCloud);

  fs::path abs_file_path;
  abs_file_path = current_path.c_str() + '/' + filename;

  ply_reader.read(filename, *cloud);

//***** Visualization ******************************************************//
//   int v1(0);
//   int v2(0);

// //  Define ViewPorts
//   pclVisualizer->createViewPort(0,0,0.5,1, v1);
//   pclVisualizer->createViewPort(0.5,0,1,1, v2);

  pclVisualizer->addPointCloud<PointT> (cloud, "cloud");

  while (!pclVisualizer->wasStopped())
    pclVisualizer->spinOnce(100);

  return 0;
}




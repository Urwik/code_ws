// cpp
#include <iostream>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <chrono>
#include <cmath>
#include <stdlib.h>
#include <experimental/filesystem>

// PCL

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_representation.h>

  // PCL FILTERS
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/normal_space.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>


//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
pcl::visualization::PCLVisualizer::Ptr pclVisualizer (new pcl::visualization::PCLVisualizer ("PCL Visualizer"));

// ************************************************************************** //
namespace fs = boost::filesystem;
int RESAMPLE_SIZE = 2;

////////////////////////////////////////////////////////////////////////////////
std::vector<double> fitScore;

int main(int argc, char **argv)
{
  // Get handlres for source and target cloud data /////////////////////////////
  PointCloud::Ptr source (new PointCloud);
  PointCloud::Ptr target (new PointCloud);

  pcl::PCDReader reader;

  auto start = std::chrono::high_resolution_clock::now();

  fs::path mydir("/home/arvc/workSpaces/data/realsense/2022.07.27");
  std::vector<std::string> filenames;
  std::string old_name;

  for(auto const& dir_entry : fs::directory_iterator(mydir)){
    if (dir_entry.path().extension() == ".pcd"){
      filenames.push_back(dir_entry.path().c_str());
    } 
  }

  std::sort(filenames.begin(), filenames.end());


//***** Visualization ******************************************************//
  int v1(0);
  int v2(0);

  //Define ViewPorts
  pclVisualizer->createViewPort(0,0,0.5,1, v1);
  pclVisualizer->createViewPort(0.5,0,1,1, v2);

  int i=0;

  while (i < filenames.size())
  {
    reader.read(filenames[i], *target);
    reader.read(filenames[i+RESAMPLE_SIZE], *source);   

    // std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(*source,*source, indices);
    // pcl::removeNaNFromPointCloud(*target,*target, indices);

    // PASSTHROUGT FILTER //
    pcl::PassThrough<PointT> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 0.75);

    pass.setInputCloud(target);
    pass.filter(*target);

    pass.setInputCloud(source);
    pass.filter(*source);

    // VOXEL FILTER //
    // pcl::VoxelGrid<PointT> sor;
    // sor.setLeafSize(0.1, 0.1, 0.1);

    // sor.setInputCloud(target);
    // sor.filter(*target);

    // sor.setInputCloud(source);
    // sor.filter(*source);


    // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(*source);
    // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(*target);
    pclVisualizer->removeAllPointClouds();

    pclVisualizer->addPointCloud<pcl::PointXYZRGB> (source, "source", v1);
    pclVisualizer->addPointCloud<pcl::PointXYZRGB> (target, "target", v2);
    
    pclVisualizer->spinOnce(100,true);

    if (pclVisualizer->wasStopped())
      break;

    i += RESAMPLE_SIZE;
  }
  
  return 0;
}




// C++
#include <iostream>
#include <stdlib.h>
#include <filesystem>
#include <algorithm>
#include <vector>
#include <math.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/file_io.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/passthrough.h>

  // PCL FILTERS
#include <pcl/filters/normal_space.h>

#include "../pointclouds/arvc_utils.cpp"

namespace fs = std::filesystem;

//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"  
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

PointCloud::Ptr readCloud(fs::path path_)
{
  PointCloud::Ptr cloud (new PointCloud);
  std::string file_ext = path_.extension();

  if (file_ext == ".pcd")
  {
    pcl::PCDReader pcd_reader;
    pcd_reader.read(path_.string(), *cloud);
  }
  else if (file_ext == ".ply")
  {
    pcl::PLYReader ply_reader;
    ply_reader.read(path_.string(), *cloud);
  }
  else
    std::cout << "Format not compatible, it should be .pcd or .ply" << std::endl;

  return cloud;
}


int check_nan_points(PointCloud::Ptr &cloud_in)
{
  int nan_count = 0;
  for (auto& point : cloud_in->points)
  {
    if( std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
      nan_count++;
  } 

  return nan_count;    
}


int main(int argc, char **argv)
{
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS); //OCULTA TODOS LOS MENSAJES DE PCL

  PointCloud::Ptr cloud_in (new PointCloud);

  fs::path current_dir = fs::current_path();

  std::cout << std::boolalpha;   
  if(argc < 2)
  {
    std::vector<fs::path> path_vector;
    for(const auto &entry : fs::directory_iterator(current_dir))
    {
      path_vector.push_back(entry.path());
    }

    for(const fs::path &entry : path_vector)
    {
      cloud_in = readCloud(entry);
      int points = check_nan_points(cloud_in);
      std::cout << entry.stem() << ": " << points << std::endl;
    }
  }

  else
  {
    std::cout << "Nan Points in Cloud:" << std::endl;
    fs::path entry = argv[1];
    cloud_in = readCloud(entry);
    int points = check_nan_points(cloud_in);
    std::cout << entry.filename().string() << ": " << points << std::endl;
  }

  return 0;
}

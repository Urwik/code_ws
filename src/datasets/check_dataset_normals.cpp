// C++
#include <iostream>
#include <stdlib.h>
#include <filesystem>
#include <algorithm>
#include <vector>

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

#include "tqdm.hpp"


namespace fs = std::filesystem;

//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"  
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"

typedef pcl::PointNormal PointT;
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


bool normals_correct(PointCloud::Ptr &cloud)
{
  for(const auto& point : cloud->points)
  {
    if(isnan(point.normal_x) || isnan(point.normal_y) || isnan(point.normal_z) )
      return false;

    else
      continue;
  }

  return true;
}



int main(int argc, char **argv)
{
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS); //OCULTA TODOS LOS MENSAJES DE PCL

  PointCloud::Ptr cloud_in (new PointCloud);

  fs::path current_dir = fs::current_path();

  std::cout << "¿Normals are Correct?:" << std::endl;
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
      std::cout << entry.stem() << ": " << normals_correct(cloud_in) << std::endl;
    }
  }

  else
  {
    fs::path entry = argv[1];
    cloud_in = readCloud(entry);
  }

  return 0;
}

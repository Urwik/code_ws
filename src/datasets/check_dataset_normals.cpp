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

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;


pcl::PointCloud<pcl::PointXYZLNormal>::Ptr readCloud(fs::path path_)
{
  pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZLNormal>);
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


int check_normals(pcl::PointCloud<pcl::PointXYZLNormal>::Ptr &cloud)
{
  // std::cout << '-'*20 << std::endl;
  int count = 0;
  for(const auto& point : cloud->points)
  {
    // std::cout << point.normal_x << std::endl;
    if(isnan(point.normal_x) || isnan(point.normal_y) || isnan(point.normal_z) )
      count++;
  }

  return count;
}


void check_labels(pcl::PointCloud<pcl::PointXYZLNormal>::Ptr &cloud)
{
  std::vector<uint32_t> labels;
  for (const auto& point : cloud->points)
  {
    labels.push_back(point.label);
  }
}


int main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZLNormal>);

  fs::path current_dir = fs::current_path();

  std::vector<int> nan_points;

  if(argc < 2)
  {
    std::vector<fs::path> path_vector;
    for(const auto &entry : fs::directory_iterator(current_dir))
    {
      path_vector.push_back(entry.path());
    }

    for(const fs::path &entry : tq::tqdm(path_vector))
    {
      cloud_in = readCloud(entry);
      nan_points.push_back(check_normals(cloud_in));
    }
  }

  else
  {
    fs::path entry = argv[1];
    cloud_in = readCloud(entry);
    check_normals(cloud_in);
  }

  float mean = 0;
  for(auto n : nan_points)
    mean += n;
  
  mean = mean / nan_points.size();

  std::cout << "\nNan Points per cloud: " << mean << std::endl;


  return 0;
}

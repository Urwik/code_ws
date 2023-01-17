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

typedef pcl::PointXYZLNormal PointT;
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


int getPlaneCount(PointCloud::Ptr &cloud)
{
  std::vector<int> labels;

  for(const auto& point : cloud->points)
    labels.push_back(point.label);

  int count;
  std::sort(labels.begin(), labels.end());
  count = std::unique(labels.begin(), labels.end()) - labels.begin();

  return count;
}

void writePlaneCount(std::vector<std::string> filename, std::vector<int> plane_count){
  std::ofstream myfile;
  myfile.open("planes_count.csv");

  for (size_t i = 0; i < filename.size(); i++)
    myfile << filename[i] << "," << plane_count[i] << "\n";

  myfile.close();
}

int main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZLNormal>);

  fs::path current_dir = fs::current_path();

  if(argc < 2)
  {
    std::vector<std::string> filenames;
    std::vector<int> plane_count;

    std::vector<fs::path> path_vector;
    for(const auto &entry : fs::directory_iterator(current_dir))
    {
      if(entry.path().extension() == ".pcd" || entry.path().extension() == ".ply")
        path_vector.push_back(entry.path());
    }

    for(const fs::path &entry : tq::tqdm(path_vector))
    {
      cloud_in = readCloud(entry);
      filenames.push_back(entry.filename().string());
      plane_count.push_back(getPlaneCount(cloud_in));
    }

    writePlaneCount(filenames, plane_count);
  }

  else
  {
    fs::path entry = argv[1];
    cloud_in = readCloud(entry);
    std::cout << "Cloud " << entry.filename().string() << " : " << getPlaneCount(cloud_in) << std::endl;
  }

  return 0;
}

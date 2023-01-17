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

namespace fs = std::filesystem;

//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;

pcl::PCDReader pcd_reader;
pcl::PLYReader ply_reader;
fs::path current_path;
std::vector<int> n_points;
std::vector<std::string> cloud_names;


int check_cloud(fs::path input_file)
{
  PointCloud::Ptr pc (new PointCloud);
  std::string file_ext = input_file.extension();
  int correct = 0;

  if (file_ext == ".pcd")
    correct = pcd_reader.read(input_file.string(), *pc);
  else if (file_ext == ".ply")
   correct =  ply_reader.read(input_file.string(), *pc);
  else
    return -1;


  if (correct == 0)
  {
    if (pc->points.size() != 25000)
      cloud_names.push_back(input_file.stem().string());

    n_points.push_back(pc->points.size());
  }
  else
    std::cout << "Error in Cloud: " << input_file.stem().string() << std::endl;

  return 0;
}



int main(int argc, char **argv)
{
  current_path = fs::current_path();

  if(argc < 2)
    for(const auto &entry : fs::directory_iterator(current_path))
    {
      check_cloud(entry.path());
    }
  else
  {
    fs::path input_dir = argv[1];
    check_cloud(input_dir);
  }

  int total = 0;

  for(int point : n_points)
    total += point;

  double mean = total / n_points.size();
  auto max = *std::max_element(n_points.begin(), n_points.end());
  auto min = *std::min_element(n_points.begin(), n_points.end());


  std::cout << "Max: " << max << std::endl;
  std::cout << "Min: " << min << std::endl;
  std::cout << "Mean: " << mean << std::endl;


  if(max != 25000 || min != 25000 || mean != 25000)
  {
    std::cout << "Dataset contains errors:" << std::endl;
    std::cout << "Clouds with not enough points" << std::endl;
    for(auto name : cloud_names)
      std::cout << name << std::endl;
  }
  else
    std::cout << "Dataset is correct" << std::endl;

  return 0;
}

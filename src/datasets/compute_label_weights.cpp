// C++
#include <iostream>
#include <stdlib.h>
#include <filesystem>
#include <algorithm>
#include <vector>

#include "../pointclouds/arvc_utils.cpp"

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


std::vector<double> computeLabelWeights(PointCloud::Ptr &cloud, bool binary)
{
  std::vector<double> label_weights;
  int cloud_size = cloud->points.size();

  // Get all labels in a cloud
  std::vector<int> labels;
  for (auto point : cloud->points)
    labels.push_back(point.label);

  // Get a vector with the different labels
  std::vector<int> different_labels;
  different_labels = labels;
  std::sort(different_labels.begin(), different_labels.end());
  auto last = std::unique(different_labels.begin(), different_labels.end());
  different_labels.erase(last, different_labels.end());

  label_weights.resize(different_labels.size());

  for (size_t i = 0; i < different_labels.size(); i++)
  {
    int count = std::count(labels.begin(), labels.end(), different_labels[i]);
    label_weights[i] = (double) count / (double) cloud_size;
  }

  if (binary == true)
  {
    std::vector<double> label_bin_weights(2);
    auto it = std::find(different_labels.begin(), different_labels.end(), 0);
    label_bin_weights[0] = label_weights[it[0]];
    label_bin_weights[1] = 1 - label_bin_weights[0];
    return label_bin_weights;
  }
  else
    return label_weights;

}



int main(int argc, char **argv)
{
  PointCloud::Ptr cloud_in (new PointCloud);
  std::vector<double> weights;
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
      cloud_in = arvc::readCloud(entry);
      weights = computeLabelWeights(cloud_in, true);
    }
    std::cout << std::endl; // '\n' after tqdm

  }

  else
  {
    fs::path entry = argv[1];
    cloud_in = arvc::readCloud(entry);
    weights = computeLabelWeights(cloud_in, true);
  }

  
  return 0;
}

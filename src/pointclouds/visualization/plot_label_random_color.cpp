#include <iostream>
#include <filesystem>
#include <thread>

#include "arvc_utils_v2.hpp"

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

using namespace std;
namespace fs = std::filesystem;

int main(int argc, char **argv)
{
  PointCloudL::Ptr cloud_in (new PointCloudL);

  fs::path current_dir = fs::current_path();

  std::cout << "Reading point cloud from: " << argv[1] << std::endl;


  fs::path pcd_file = current_dir / argv[1];
  cloud_in = arvc::readPointCloud<PointL>(pcd_file.string());  
  
  std::cout << "Reading point cloud from: " << pcd_file.string() << std::endl;



  std::vector<int> single_labels;

  for (size_t i = 0; i < cloud_in->points.size(); i++) {
    int label = cloud_in->points[i].label;

    // If the label is not in the used_labels vector, add it
    if (std::find(single_labels.begin(), single_labels.end(), label) == single_labels.end()) {
      single_labels.push_back(label);
    }
  }


  pcl::visualization::PCLVisualizer my_vis;

  my_vis.setBackgroundColor(1,1,1);

  for (int label : single_labels) {
    PointCloudL::Ptr cloud (new PointCloudL);
    pcl::PointIndices::Ptr indices (new pcl::PointIndices);

    for (size_t i = 0; i < cloud_in->points.size(); i++) {
      if (cloud_in->points[i].label == label) {
        indices->indices.push_back(i);
      }
    }

    pcl::ExtractIndices<PointL> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*cloud);

    if (cloud->points.size() == 0) {
      continue;
    }

    int r;
    int g;
    int b;

    if (label == 0) {
      r = 10;
      g = 10;
      b = 10;

    }
    else {
      r = rand() % 255;
      g = rand() % 255;
      b = rand() % 255;
    }
    

    std::cout << "R: " << r << " G: " << g << " B: " << b << std::endl;



    pcl::visualization::PointCloudColorHandlerCustom<PointL> color (cloud, r, g, b);
    my_vis.addPointCloud(cloud, color, "cloud" + std::to_string(label));

  }

  while (!my_vis.wasStopped()) {
    my_vis.spinOnce(100);
  }
  return 0;  
}



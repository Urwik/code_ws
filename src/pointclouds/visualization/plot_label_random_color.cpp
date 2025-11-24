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

/**
 * Displays usage information for the program
 */
void display_usage() {
  std::cout << "Usage: plot_label_random_color <point_cloud_file>" << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "  -h, --help    Show this help message and exit" << std::endl;
  std::cout << std::endl;
  std::cout << "Description:" << std::endl;
  std::cout << "  Visualizes a point cloud with random colors assigned to each label." << std::endl;
  std::cout << "  Each unique label in the point cloud will be displayed with a different color." << std::endl;
  std::cout << "  Label 0 will always be shown in black." << std::endl;
}

using namespace std;
namespace fs = std::filesystem;

int main(int argc, char **argv)
{
  // Check for help flag or incorrect input
  if (argc < 2 || strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0) {
    display_usage();
    return 0;
  }

  PointCloudL::Ptr cloud_in (new PointCloudL);

  fs::path current_dir = fs::current_path();

  std::cout << "Reading point cloud from: " << argv[1] << std::endl;

  // Check if file exists
  fs::path pcd_file = current_dir / argv[1];
  if (!fs::exists(pcd_file)) {
    std::cerr << "Error: File " << pcd_file.string() << " does not exist" << std::endl;
    display_usage();
    return 1;
  }

  try {
    cloud_in = arvc::readPointCloud<PointL>(pcd_file.string());
    std::cout << "Point cloud loaded successfully with " << cloud_in->points.size() << " points" << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Error loading point cloud: " << e.what() << std::endl;
    return 1;
  }

  // Check if cloud has any points
  if (cloud_in->points.empty()) {
    std::cerr << "Error: Point cloud is empty" << std::endl;
    return 1;
  }
  
  std::cout << "Identifying unique labels in the point cloud..." << std::endl;

  std::vector<int> single_labels;

  for (size_t i = 0; i < cloud_in->points.size(); i++) {
    int label = cloud_in->points[i].label;

    // If the label is not in the used_labels vector, add it
    if (std::find(single_labels.begin(), single_labels.end(), label) == single_labels.end()) {
      single_labels.push_back(label);
    }
  }

  std::cout << "Found " << single_labels.size() << " unique labels" << std::endl;

  pcl::visualization::PCLVisualizer my_vis;

  my_vis.setBackgroundColor(1,1,1);
  my_vis.addCoordinateSystem(1.0);
  my_vis.setWindowName("Point Cloud Visualization by Label");

  std::cout << "Visualizing point cloud with labels..." << std::endl;

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
      r = 100;
      g = 100;
      b = 100;

    }
    else {
      r = rand() % 255;
      g = rand() % 255;
      b = rand() % 255;
    }
    

    std::cout << "R: " << r << " G: " << g << " B: " << b << std::endl;



    pcl::visualization::PointCloudColorHandlerCustom<PointL> color (cloud, r, g, b);
    my_vis.addPointCloud(cloud, color, "cloud" + std::to_string(label));
    my_vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud" + std::to_string(label));
  }

  std::cout << "Visualization ready. Press 'q' to close the window." << std::endl;

  while (!my_vis.wasStopped()) {
    my_vis.spinOnce(100);
  }
  return 0;  
}



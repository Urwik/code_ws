// C++
#include <iostream>
#include <filesystem>
#include <thread>
#include <cstring>

#include "arvc_utils_v2.hpp"

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/filter.h>
#include <cmath>

// Type Definitions ////////////////////////////////////////////////////////////
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudWithNormals;

namespace fs = std::filesystem;
using namespace std;

/**
 * Displays usage information for the program
 */
void display_usage() {
  cout << "Usage: plot_by_curvature <point_cloud_file>" << endl;
  cout << "Options:" << endl;
  cout << "  -h, --help    Show this help message and exit" << endl;
  cout << endl;
  cout << "Description:" << endl;
  cout << "  Computes and visualizes the curvature of a point cloud using color gradients." << endl;
  cout << "  Red indicates high curvature, green indicates low curvature." << endl;
  cout << "  Supported formats: .pcd, .ply" << endl;
}

PointCloud::Ptr readCloud(fs::path path_)
{
  PointCloud::Ptr cloud (new PointCloud);
  string file_ext = path_.extension();

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
    cout << "Format not compatible, it should be .pcd or .ply" << endl;

  return cloud;
}


vector<int> colorGradient(float value)
{
  // Handle invalid values
  if (isnan(value) || isinf(value)) {
    value = 0.0f;
  }
  
  // Clamp value to [0, 1] range
  value = max(0.0f, min(1.0f, value));

  if (value < 0.01){
    value = 0.0f;
    return {100, 100, 100}; // green
  }


  // value = value*value;  // Emphasize lower curvature values
  // ephasize hihger curvature values
  value = 1.0f - (1.0f - value) * (1.0f - value);

  vector<int> color;
  int red = (int)(value * 255); // high curvature -> red
  int green = (int)((1 - value) * 255); // low curvature -> green
  int blue = 0;

  // Ensure color values are in valid range [0, 255]
  red = max(0, min(255, red));
  green = max(0, min(255, green));

  color = {red, green, blue};

  return color;
}

PointCloudWithNormals::Ptr computeNormalsAndCurvature(PointCloud::Ptr cloud, int neighbors=30, double radius = 0.0)
{
  cout << "Computing normals and curvature..." << endl;
  
  // Remove invalid points (NaN, inf)
  PointCloud::Ptr cloud_filtered(new PointCloud);
  vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud_filtered, indices);
  cout << "Removed " << (cloud->points.size() - cloud_filtered->points.size()) << " invalid points" << endl;
  
  if (cloud_filtered->points.empty()) {
    cerr << "Error: No valid points after filtering" << endl;
    return PointCloudWithNormals::Ptr();
  }

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<PointT, PointNT> normal_estimation;
  normal_estimation.setInputCloud(cloud_filtered);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
  normal_estimation.setSearchMethod(tree);

  // Output datasets
  PointCloudWithNormals::Ptr cloud_normals(new PointCloudWithNormals);

  // Try different approaches for neighbor search
  // First try with K nearest neighbors
  normal_estimation.setKSearch(neighbors);  // Use 20 nearest neighbors
  normal_estimation.setRadiusSearch(radius);  // Disable radius search
  
  // Compute the features
  normal_estimation.compute(*cloud_normals);
  
  // // If many invalid normals, try radius search instead
  // int invalid_normals = 0;
  // for (size_t i = 0; i < cloud_normals->points.size(); ++i) {
  //   if (isnan(cloud_normals->points[i].normal_x) || 
  //       isnan(cloud_normals->points[i].normal_y) || 
  //       isnan(cloud_normals->points[i].normal_z) ||
  //       isnan(cloud_normals->points[i].curvature)) {
  //     invalid_normals++;
  //   }
  // }
  
  // cout << "Invalid normals/curvatures with K-search: " << invalid_normals << " out of " << cloud_normals->points.size() << endl;
  
  // // If too many invalid normals, try radius search
  // if (invalid_normals > cloud_normals->points.size() * 0.5) {
  //   cout << "Too many invalid normals, trying radius search..." << endl;
  //   normal_estimation.setKSearch(0);  // Disable K search
  //   normal_estimation.setRadiusSearch(radius);  // Use radius search
  //   normal_estimation.compute(*cloud_normals);
    
  //   // Count invalid normals again
  //   invalid_normals = 0;
  //   for (size_t i = 0; i < cloud_normals->points.size(); ++i) {
  //     if (isnan(cloud_normals->points[i].normal_x) || 
  //         isnan(cloud_normals->points[i].normal_y) || 
  //         isnan(cloud_normals->points[i].normal_z) ||
  //         isnan(cloud_normals->points[i].curvature)) {
  //       invalid_normals++;
  //     }
  //   }
  //   cout << "Invalid normals/curvatures with radius search: " << invalid_normals << " out of " << cloud_normals->points.size() << endl;
  // }

  // Copy XYZ coordinates from filtered cloud
  for (size_t i = 0; i < cloud_filtered->points.size() && i < cloud_normals->points.size(); ++i) {
    cloud_normals->points[i].x = cloud_filtered->points[i].x;
    cloud_normals->points[i].y = cloud_filtered->points[i].y;
    cloud_normals->points[i].z = cloud_filtered->points[i].z;
  }

  // Validate curvature values
  int nan_count = 0;
  int valid_count = 0;
  for (size_t i = 0; i < cloud_normals->points.size(); ++i) {
    if (isnan(cloud_normals->points[i].curvature) || isinf(cloud_normals->points[i].curvature)) {
      cloud_normals->points[i].curvature = 0.0f;  // Set invalid curvatures to 0
      nan_count++;
    } else {
      valid_count++;
    }
  }

  cout << "Normals and curvature computed for " << cloud_normals->points.size() << " points" << endl;
  cout << "Valid curvature values: " << valid_count << ", Invalid (set to 0): " << nan_count << endl;
  
  return cloud_normals;
}


void plotCloud(PointCloudWithNormals::Ptr &cloud_with_normals)
{
  cout << "Creating curvature visualization..." << endl;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr gradient_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  // Extract valid curvature values
  vector<float> valid_curvatures;
  for (size_t i = 0; i < cloud_with_normals->points.size(); i++) {
    float curv = cloud_with_normals->points[i].curvature;
    if (!isnan(curv) && !isinf(curv)) {
      valid_curvatures.push_back(curv);
    }
  }

  if (valid_curvatures.empty()) {
    cerr << "Error: No valid curvature values found!" << endl;
    return;
  }

  // Calculate statistics
  float min_curv = *min_element(valid_curvatures.begin(), valid_curvatures.end());
  float max_curv = *max_element(valid_curvatures.begin(), valid_curvatures.end());
  
  cout << "Valid curvature values: " << valid_curvatures.size() << " out of " << cloud_with_normals->points.size() << endl;
  cout << "Maximum curvature value: " << max_curv << endl;
  cout << "Minimum curvature value: " << min_curv << endl;

  // Handle edge cases for normalization
  float curv_range = max_curv - min_curv;
  if (curv_range <= 0) {
    cout << "Warning: All curvature values are the same, using uniform coloring" << endl;
    curv_range = 1.0f;  // Avoid division by zero
  }

  gradient_cloud->resize(cloud_with_normals->size());

  // Apply color gradient based on curvature
  for (size_t i = 0; i < cloud_with_normals->points.size(); i++)
  {
    vector<int> color;
    
    float curv = cloud_with_normals->points[i].curvature;
    float normalized_curv;
    
    // Normalize curvature value
    if (isnan(curv) || isinf(curv)) {
      normalized_curv = 0.0f;  // Invalid values get minimum color
    } else {
      normalized_curv = (curv - min_curv) / curv_range;
      // Clamp to [0, 1] range
      normalized_curv = max(0.0f, min(1.0f, normalized_curv));
    }
    
    color = colorGradient(normalized_curv);
    
    gradient_cloud->points[i].x = cloud_with_normals->points[i].x;
    gradient_cloud->points[i].y = cloud_with_normals->points[i].y;
    gradient_cloud->points[i].z = cloud_with_normals->points[i].z;
    gradient_cloud->points[i].r = color[0];
    gradient_cloud->points[i].g = color[1];
    gradient_cloud->points[i].b = color[2];
  }

  cout << "Starting visualization. Press 'q' to close the window." << endl;

  pcl::visualization::PCLVisualizer visualizer("Curvature Visualization");
  visualizer.setBackgroundColor(1.0, 1.0, 1.0);
  // visualizer.addCoordinateSystem(1.0);
  visualizer.addPointCloud<pcl::PointXYZRGB>(gradient_cloud, "curvature_cloud");
  visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "curvature_cloud");

  while (!visualizer.wasStopped())
    visualizer.spinOnce(100);
}



int main(int argc, char **argv)
{
  // Check for help flag or incorrect input
  if (argc < 2 || strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0) {
    display_usage();
    return 0;
  }

  fs::path current_path = fs::current_path();
  PointCloud::Ptr cloud_in(new PointCloud);

  fs::path entry = argv[1];
  
  // Check if file exists
  if (!fs::exists(entry)) {
    cerr << "Error: File " << entry.string() << " does not exist" << endl;
    display_usage();
    return 1;
  }

  try {
    cout << "Loading point cloud from: " << entry.string() << endl;
    cloud_in = readCloud(entry);
    
    if (cloud_in->points.empty()) {
      cerr << "Error: Point cloud is empty" << endl;
      return 1;
    }
    
    cout << "Point cloud loaded successfully with " << cloud_in->points.size() << " points" << endl;
    
    // Compute normals and curvature
    PointCloudWithNormals::Ptr cloud_with_normals = computeNormalsAndCurvature(cloud_in, 50, 0.0);
    
    // Visualize the curvature
    plotCloud(cloud_with_normals);
    
  } catch (const exception& e) {
    cerr << "Error processing point cloud: " << e.what() << endl;
    return 1;
  }

  return 0;
}



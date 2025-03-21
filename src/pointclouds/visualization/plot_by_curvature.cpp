#include <iostream>
#include <filesystem>
#include <thread>

#include "arvc_utils.cpp"

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>

// Type Definitions ////////////////////////////////////////////////////////////
typedef pcl::PointXYZLNormal PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace fs = std::filesystem;


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


std::vector<int> colorGradient(float value)
{
  std::vector<int> color;
  int red = (int) value*255;
  int green = (int) (1 - value)*255;
  int blue = 0;

  color = {red, green, blue};

  std::cout << red << ',' << green << ',' << blue << endl;

  return color;
}


void plotCloud(PointCloud::Ptr &cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr gradient_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  Eigen::VectorXf curv_vec(cloud->points.size());

  for (size_t i = 0; i < cloud->points.size(); i++)
    curv_vec(i)= cloud->points[i].curvature;

  std::cout << "Maximo valor de Curvatura: " << curv_vec.maxCoeff() << std::endl;
  std::cout << "Minimo valor de Curvatura: " << curv_vec.minCoeff() << std::endl;


  // Eigen::VectorXf curv_norm;
  // curv_norm = curv_vec / curv_vec.maxCoeff();

  // gradient_cloud->resize(cloud->size());

  // for (size_t i = 0; i < cloud->points.size(); i++)
  // {
  //   std::vector<int> color;
  //   // std::cout << curv_norm(i) << std::endl;
  //   color = colorGradient(curv_norm(i));
  //   gradient_cloud->points[i].x = cloud->points[i].x;
  //   gradient_cloud->points[i].y = cloud->points[i].y;
  //   gradient_cloud->points[i].z = cloud->points[i].z;
  //   gradient_cloud->points[i].r = color[0];
  //   gradient_cloud->points[i].g = color[1];
  //   gradient_cloud->points[i].b = color[2];
  // }

  // pcl::visualization::PCLVisualizer visualizer;
  // visualizer.addPointCloud<pcl::PointXYZRGB>(gradient_cloud, "curvature_cloud");

  // while (!visualizer.wasStopped())
  //   visualizer.spinOnce(100);

}



int main(int argc, char **argv)
{
  fs::path current_path = fs::current_path();
  PointCloud::Ptr cloud_in (new PointCloud);

  fs::path entry = argv[1];
  cloud_in = arvc::readCloud(entry);
  plotCloud(cloud_in);

  return 0;  
}



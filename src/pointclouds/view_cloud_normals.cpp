// C++
#include <iostream>
#include <stdlib.h>
#include <filesystem>


// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "tqdm.hpp"

//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

typedef pcl::PointXYZLNormal PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// ************************************************************************** //
namespace fs = std::filesystem;

////////////////////////////////////////////////////////////////////////////////

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

pcl::PointCloud<pcl::Normal>::Ptr getNormals(PointCloud::Ptr &cloud_in)
{
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  cloud_normals->points.resize(cloud_in->points.size());

  for (size_t i = 0; i < cloud_in->points.size(); i++)
  {
    cloud_normals->points[i].normal_x = cloud_in->points[i].normal_x;
    cloud_normals->points[i].normal_y = cloud_in->points[i].normal_y;
    cloud_normals->points[i].normal_z = cloud_in->points[i].normal_z;
    cloud_normals->points[i].curvature = cloud_in->points[i].curvature;
  }

  return cloud_normals;  
}



void plotCloud(PointCloud::Ptr &cloud_in)
{
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  cloud_normals = getNormals(cloud_in);

  pcl::visualization::PCLVisualizer vis;
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color (cloud_in, 0, 155, 0);

  vis.addPointCloud<PointT>(cloud_in, cloud_color, "cloud");
  vis.addPointCloudNormals<PointT, pcl::Normal>(cloud_in, cloud_normals, 30, 0.1, "normals");

  while (!vis.wasStopped())
    vis.spinOnce(100);
  
}



int main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZLNormal>);
  fs::path current_dir = fs::current_path();

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
      plotCloud(cloud_in);
    }
  }

  else
  {
    fs::path entry = argv[1];
    cloud_in = readCloud(entry);
    plotCloud(cloud_in);
  }

  return 0;
}

// C++
#include <iostream>
#include <algorithm>
#include <filesystem>

// PCL
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include "tqdm.hpp"

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"  
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////
typedef pcl::PointXYZ PointT;
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


pcl::PointIndices::Ptr range_filter(PointCloud::Ptr &cloud_in_, float min_range_, float max_range_)
{
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  float range;
  for (size_t i = 0; i < cloud_in_->points.size(); i++)
  {
    range = pcl::squaredEuclideanDistance(pcl::PointXYZ(0,0,0), cloud_in_->points[i]);
    if (range > min_range_ && range < max_range_ )
      inliers->indices.push_back(i);
  }
  
  return inliers;
}


////////////////////////////////////////////////////////////////////////////////
  pcl::PointCloud<pcl::PointXYZ>::Ptr 
  extractIndices(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
    pcl::PointIndices::Ptr &indices)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(indices);
    extract.filter(*cloud_out);

    return cloud_out;
  }


void writeCloud(PointCloud::Ptr &cloud_in, fs::path entry)
{
  pcl::PLYWriter ply_writer;
  
  fs::path abs_file_path = fs::current_path().parent_path() / "ply_xyzsampled";
  if (!fs::exists(abs_file_path)) 
    fs::create_directory(abs_file_path);

  std::string filename = entry.stem().string() + ".ply";
  
  abs_file_path = abs_file_path / filename;
  ply_writer.write(abs_file_path, *cloud_in, true, false);
}


int main(int argc, char **argv)
{
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS); //OCULTA TODOS LOS MENSAJES DE PCL

  PointCloud::Ptr cloud_in (new PointCloud);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  PointCloud::Ptr cloud_out (new PointCloud);

  fs::path current_dir = fs::current_path();

  if(argc < 2)
  {
    std::vector<fs::path> path_vector;
    for(const auto &entry : fs::directory_iterator(current_dir))
    {
      if(entry.path().extension() == ".pcd" || entry.path().extension() == ".ply")
        path_vector.push_back(entry.path());
    }

    for(const fs::path &entry : tq::tqdm(path_vector))
    {
      cloud_in = readCloud(entry);
      inliers = range_filter(cloud_in, 0.2, 20.0);
      cloud_out = extractIndices(cloud_in, inliers);
      writeCloud(cloud_out, entry);

    }


  }
  else
  {
    fs::path entry = argv[1];
    cloud_in = readCloud(entry);
    inliers = range_filter(cloud_in, 0.2, 20.0);
    cloud_out = extractIndices(cloud_in, inliers);
    writeCloud(cloud_out, entry);
  }

  std::cout << GREEN << "COMPLETED!!" << RESET << std::endl;
  return 0;
}




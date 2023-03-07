// C++
#include <iostream>
#include <algorithm>
#include <filesystem>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>

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


pcl::PointCloud<pcl::PointNormal>::Ptr computeNormals(PointCloud::Ptr &cloud_in)
{
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  
  ne.setInputCloud(cloud_in);
  ne.setSearchMethod(tree);
  ne.setKSearch(20);
  // ne.setRadiusSearch(0.1);
  ne.compute(*cloud_normals);

  pcl::concatenateFields(*cloud_in, *cloud_normals, *cloud_out); 

  return cloud_out;
}


pcl::PointCloud<pcl::PointNormal>::Ptr computeNormals2(PointCloud::Ptr &cloud_in)
{
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out (new pcl::PointCloud<pcl::PointNormal>);
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  pcl::NormalEstimation<PointT, pcl::PointNormal> ne;
  
  ne.setInputCloud(cloud_in);
  ne.setSearchMethod(tree);
  ne.setKSearch(30);
  // ne.setRadiusSearch(0.1);
  ne.compute(*cloud_out);

  return cloud_out;
}


void writeCloud(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_in, fs::path entry)
{
  pcl::PLYWriter ply_writer;
  
  fs::path abs_file_path = fs::current_path().parent_path() / "ply_xyznormal";
  if (!fs::exists(abs_file_path)) 
    fs::create_directory(abs_file_path);

  std::string filename = entry.stem().string() + ".ply";

  abs_file_path = abs_file_path / filename;
  ply_writer.write(abs_file_path.string(), *cloud_in, true, false);
}


int main(int argc, char **argv)
{
  auto start = std::chrono::high_resolution_clock::now();
  PointCloud::Ptr cloud_in (new PointCloud);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out (new pcl::PointCloud<pcl::PointNormal>);

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
      cloud_out = computeNormals(cloud_in);
      writeCloud(cloud_out, entry);
    }
  }
  else
  {
    fs::path entry = argv[1];
    cloud_in = readCloud(entry);
    // cloud_in_label = parseToXYZLabel(cloud_in);
    cloud_out = computeNormals(cloud_in);
    // COMPUTATION TIME
    writeCloud(cloud_out, entry);
  }

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "Computation Time: " << duration.count() << " ms" << std::endl;
  
  std::cout << GREEN << "COMPLETED!!" << RESET << std::endl;
  return 0;
}




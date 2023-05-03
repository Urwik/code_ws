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


pcl::PointCloud<pcl::PointXYZLNormal>::Ptr computeNormals(pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud_in)
{
  pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZLNormal>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZL>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZL> ());
  pcl::NormalEstimation<pcl::PointXYZL, pcl::Normal> ne;
  
  ne.setInputCloud(cloud_in);
  ne.setSearchMethod(tree);
  ne.setKSearch(10);
  // ne.setRadiusSearch(0.1);
  ne.compute(*cloud_normals);

  pcl::concatenateFields(*cloud_in, *cloud_normals, *cloud_out); 

  return cloud_out;
}


pcl::PointCloud<pcl::PointXYZL>::Ptr parseToXYZLabel(PointCloud::Ptr &cloud_in)
{
  pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZL>);
  cloud_out->points.resize(cloud_in->points.size());

  for (size_t i = 0; i < cloud_in->size(); i++)
  {
    cloud_out->points[i].x = cloud_in->points[i].x;
    cloud_out->points[i].y = cloud_in->points[i].y;
    cloud_out->points[i].z = cloud_in->points[i].z;
    cloud_out->points[i].label = (uint32_t) cloud_in->points[i].intensity;
  }

  return cloud_out;  
}


void writeCloud(pcl::PointCloud<pcl::PointXYZLNormal>::Ptr &cloud_in, fs::path entry)
{
  pcl::PCDWriter pcd_writer;
  
  fs::path abs_file_path = fs::current_path().parent_path() / "pcd_xyzlabelnormal";
  if (!fs::exists(abs_file_path)) 
    fs::create_directory(abs_file_path);

  std::string filename = entry.stem().string() + ".pcd";

  abs_file_path = abs_file_path / filename;
  pcd_writer.write(abs_file_path.string(), *cloud_in, true);
}


int main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_in_label (new pcl::PointCloud<pcl::PointXYZL>);
  pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZLNormal>);

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
      std::cout << entry.filename() << std::endl;
      cloud_in = readCloud(entry);
      cloud_in_label = parseToXYZLabel(cloud_in);
      cloud_out = computeNormals(cloud_in_label);
      writeCloud(cloud_out, entry);
    }


  }
  else
  {
    fs::path entry = argv[1];
    cloud_in = readCloud(entry);
    cloud_in_label = parseToXYZLabel(cloud_in);
    cloud_out = computeNormals(cloud_in_label);
    writeCloud(cloud_out, entry);
  }

  std::cout << GREEN << "COMPLETED!!" << RESET << std::endl;
  return 0;
}




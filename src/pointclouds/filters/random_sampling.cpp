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
#include <pcl/filters/random_sample.h>

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


PointCloud::Ptr randomSample(PointCloud::Ptr &cloud_in)
{
  PointCloud::Ptr out_cloud (new PointCloud);
  pcl::RandomSample<PointT> rs;
  rs.setInputCloud(cloud_in);
  rs.setSample(25000);
  rs.filter(*out_cloud);

  return out_cloud;
}


void writeCloud(PointCloud::Ptr &cloud_in, fs::path entry)
{
  pcl::PLYWriter ply_writer;
  
  fs::path abs_file_path = fs::current_path().parent_path() / "ply_xyzsampled";
  if (!fs::exists(abs_file_path)) 
    fs::create_directory(abs_file_path);

  std::string filename = entry.stem().string() + ".ply";
  
  abs_file_path = abs_file_path / filename;
  ply_writer.write(abs_file_path, *cloud_in, true);
}


int main(int argc, char **argv)
{
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS); //OCULTA TODOS LOS MENSAJES DE PCL

  PointCloud::Ptr cloud_in (new PointCloud);
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
      cloud_out = randomSample(cloud_in);
      writeCloud(cloud_out, entry);

    }


  }
  else
  {
    fs::path entry = argv[1];
    cloud_in = readCloud(entry);
    cloud_out = randomSample(cloud_in);
    writeCloud(cloud_out, entry);
  }

  std::cout << GREEN << "COMPLETED!!" << RESET << std::endl;
  return 0;
}




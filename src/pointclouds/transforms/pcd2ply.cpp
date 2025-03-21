// cpp
#include <iostream>
#include <stdlib.h>
#include <filesystem>

// PCL
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

  // PCL FILTERS
#include "tqdm.hpp"

namespace fs = std::filesystem;

//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

typedef pcl::PointXYZL PointL;
typedef pcl::PointCloud<PointL> PointCloud;

PointCloud::Ptr readCloud(fs::path path_)
{
  PointCloud::Ptr cloud (new PointCloud);
  std::string file_ext = path_.extension();

  if (file_ext == ".pcd") {
    pcl::PCDReader pcd_reader;
    pcd_reader.read(path_.string(), *cloud);
  }

  return cloud;
}


void pcd_to_ply(PointCloud::Ptr &cloud, const std::string filename, bool binary = true)
{
  std::string out_filename;
  fs::path out_file_path;
  fs::path current_path = fs::current_path();
  fs::path ply_path = current_path.parent_path() / "ply_xyzlabelnormal";
  if(!fs::exists(ply_path))
  fs::create_directory(ply_path);

  out_filename = filename + ".ply";
  out_file_path = ply_path / out_filename;

  pcl::PLYWriter ply_writer;
  ply_writer.write(out_file_path.string(), *cloud, binary, false);
}


int main(int argc, char **argv)
{
  fs::path current_dir = fs::current_path();

  PointCloud::Ptr cloud (new PointCloud);
  std::string filename;


  if(argc < 2)
  {
    std::vector<fs::path> path_vector;
    for(const auto &entry : fs::directory_iterator(current_dir))
    {
      if(entry.path().extension() == ".pcd")
        path_vector.push_back(entry.path());
    }

    for(const fs::path &entry : tq::tqdm(path_vector))
    {
      filename = entry.stem().string();
      cloud = readCloud(entry);
      pcd_to_ply(cloud, filename);
    }
  }
  else
  {
    fs::path entry = argv[1];
    cloud = readCloud(entry);
    pcd_to_ply(cloud, entry.stem().string());
  }

  return 0;
}
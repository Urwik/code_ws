#pragma once

#include <iostream>
#include <filesystem>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>

namespace fs = std::filesystem;

template<typename PointT>
void writeCloud(typename pcl::PointCloud<PointT>::Ptr &cloud_in, fs::path entry, std::string folder_name = "ply_xyzl_fixedSize")
{
  pcl::PLYWriter ply_writer;
  
  fs::path dir_path = fs::absolute(entry).parent_path().parent_path() / folder_name;

  if (!fs::exists(dir_path)) 
    fs::create_directory(dir_path);

  std::string filename = entry.stem().string() + ".ply";
  
  fs::path abs_file_path = dir_path / filename;
  ply_writer.write(abs_file_path, *cloud_in, true);
}
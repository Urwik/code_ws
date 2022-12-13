// cpp
#include <iostream>
#include <stdlib.h>
#include <filesystem>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/file_io.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/passthrough.h>

  // PCL FILTERS
#include <pcl/filters/normal_space.h>

namespace fs = std::filesystem;

//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;

pcl::PLYReader ply_reader;
pcl::PLYWriter ply_writer;
pcl::RandomSample<PointT> rand_filter;
fs::path current_path;

void plot_label(fs::path input_file)
{
  PointCloud::Ptr pc (new PointCloud);

  //DIRECTORIES
  std::string file_ext, in_filename, out_filename, out_filename_path;

  file_ext = input_file.extension();

  //READ AND WRITE

  if (file_ext == ".ply"){
    ply_reader.read(input_file.string(), *pc);
    for (auto& point : *pc)
    {
      std::cout << point.intensity;
    }
  }

  std::cout << std::endl;
}

int main(int argc, char **argv)
{
  current_path = fs::current_path();
  
  if(argc < 2)
    for(const auto &entry : fs::directory_iterator(current_path))
    {
      plot_label(entry.path());
      // read_file(entry.path());
    }
  else
  {
    fs::path input_dir = argv[1];
    plot_label(input_dir);
  }

  return 0;
}

// cpp
#include <iostream>
#include <stdlib.h>
#include <filesystem>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/passthrough.h>

  // PCL FILTERS
#include <pcl/filters/normal_space.h>

namespace fs = std::filesystem;

//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

pcl::PCDReader pcd_reader;
pcl::PLYWriter ply_writer;
pcl::RandomSample<PointT> rand_filter;
pcl::PassThrough<PointT> pass;
fs::path current_path;

void pcd_to_ply(fs::path input_file, std::string output_dir = "/ply", bool filter = false, int npoints = 4000, bool binary = true)
{

  std::vector<std::string> axis_to_filter = {"x", "y", "z"};
  rand_filter.setSample((unsigned int) npoints);
  rand_filter.setSeed(std::rand());

  std::string file_ext, in_filename, out_filename, out_filename_path;
  PointCloud::Ptr pc (new PointCloud);

  file_ext = input_file.extension();
  in_filename = input_file.stem();
  out_filename = in_filename + ".ply";
  out_filename_path = current_path / output_dir / out_filename;

  if (file_ext == ".pcd"){
    pcd_reader.read(input_file.string(), *pc);
    if (filter){
      rand_filter.setInputCloud(pc);
      rand_filter.filter(*pc);
    }

    ply_writer.write(out_filename_path, *pc, binary, true);
  }
}


int main(int argc, char **argv)
{
  current_path = fs::current_path();
  
  if(argc < 2)
    for(const auto &entry : fs::directory_iterator(current_path))
    {
      pcd_to_ply(entry.path(), "/ply", true, 30000, true);
    }
  else
  {
    fs::path input_dir = argv[1];
    pcd_to_ply(input_dir, "/ply", true, 30000, true);
  }

  return 0;
}




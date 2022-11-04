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

void pcd2ply(std::string input_dir, std::string output_dir = "/ply/", bool filter = false, int npoints = 4000, bool binary = true){
    
    pcl::PCDReader pcd_reader;
    pcl::PLYWriter ply_writer;
    pcl::RandomSample<PointT> rand_filter;
    pcl::PassThrough<PointT> pass;
    std::vector<std::string> axis_to_filter = {"x", "y", "z"};
    
    rand_filter.setSample((unsigned int) npoints);
    rand_filter.setSeed(std::rand());


    std::string file_ext, in_filename, out_filename, out_filename_path;
    PointCloud::Ptr pc (new PointCloud);

  for (const auto & entry : fs::directory_iterator(input_dir))
  {
    file_ext = entry.path().filename().extension();
    in_filename = entry.path().stem(); //filename without extension
    out_filename.clear();
    out_filename = in_filename.append(".ply");
    out_filename_path = output_dir;
    out_filename_path = out_filename_path.append(out_filename);;

    if (file_ext == ".pcd"){
      pcd_reader.read(entry.path().string(), *pc);
      
      if (filter){

        for (std::string axis : axis_to_filter)
        {
          pass.setInputCloud(pc);
          pass.setFilterFieldName (axis);
          pass.setFilterLimits (-2.0, 2.0);
          pass.filter(*pc) ;
        }

        rand_filter.setInputCloud(pc);
        rand_filter.filter(*pc);

      }

      ply_writer.write(out_filename_path, *pc, binary, true);
    }
  }
}



int main(int argc, char **argv)
{
  std::string ROOT_DIR, OUTPUT_DIR;
  
  ROOT_DIR = "/media/arvc/data/experiments/realsense/real/2022.07.27/raw/";
  OUTPUT_DIR = "/media/arvc/data/experiments/realsense/real/2022.07.27/filtered/";

  fs::path out_dir = OUTPUT_DIR;
  fs::path root_dir = ROOT_DIR;

  if (!fs::exists(out_dir))
  {
    fs::create_directory(out_dir);
  }
  

  pcd2ply(ROOT_DIR, OUTPUT_DIR, true, 100000, true);
  std::cout << "DONE!" << std::endl;

  return 0;
}




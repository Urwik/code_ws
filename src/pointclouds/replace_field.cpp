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

void rename_field(fs::path input_file, std::string output_dir = "/bin_class/")
{
  std::cout << "Parsing file " << input_file.filename() << std::endl;
  //CLOUD
  PointCloud::Ptr pc (new PointCloud);

  //DIRECTORIES
  std::string file_ext, in_filename, out_filename, out_filename_path;

  file_ext = input_file.extension();
  in_filename = input_file.stem();
  out_filename = in_filename + ".ply";
  std::string out_dir = current_path.string() + output_dir;
  std::cout << fs::path(out_dir).string() << std::endl;

  if(!fs::exists(fs::path(out_dir)))
    fs::create_directory(fs::path(out_dir));

  out_filename_path = out_dir + out_filename;

  //READ AND WRITE

  if (file_ext == ".ply"){
    ply_reader.read(input_file.string(), *pc);
    for (auto& point : *pc)
    {
      if(point.intensity > 0)
        point.intensity = 1;
    }

    ply_writer.write(out_filename_path, *pc, true, true);
  }
}

void read_file (fs::path input_file){
  PointCloud::Ptr pc (new PointCloud);
  pcl::io::loadPCDFile(input_file, *pc);

  std::cout << pc->width << std::endl;
  std::cout << pc->points.size() << std::endl;
}



int main(int argc, char **argv)
{
  current_path = fs::current_path();
  
  if(argc < 2)
    for(const auto &entry : fs::directory_iterator(current_path))
    {
      rename_field(entry.path(), "/bin_class/");
      // read_file(entry.path());
    }
  else
  {
    fs::path input_dir = argv[1];
    rename_field(input_dir, "/bin_class/");
  }

  return 0;
}



  // while (!infile.eof())
  // {
  //   std::getline(infile, line);
  //   std::cout << line << std::endl;
  // }

  // for (size_t i = 0; i < 7; i++)
  // {
  //   std::getline(infile, line);
  //   6,9
  // }

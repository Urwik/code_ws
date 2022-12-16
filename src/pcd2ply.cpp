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

pcl::PCDReader pcd_reader;
pcl::PLYWriter ply_writer;
pcl::RandomSample<PointT> rand_filter;
pcl::PassThrough<PointT> pass;
fs::path current_path;
fs::path ply_path;

void read_txt(fs::path input_file)
{
  std::cout << "Fixing error on " << input_file.filename() << std::endl;
  std::ifstream infile(input_file);
  std::string orig_file_name = input_file.filename();
  fs::path file_root_path = input_file.parent_path();
  fs::path tmp_file_path = file_root_path / "tmp.pcd";
  std::ofstream outfile("tmp.pcd");
  std::string line;

  int line_count = 0;
  while(!infile.eof())
  {
    std::getline(infile, line);

    if(line_count == 6)
    {
      std::string tmp_string = line.substr(6,5);
      // std::cout << tmp_string << std::endl;
      int tmp_int = std::stoi(tmp_string);
      tmp_int = tmp_int - 1;
      tmp_string = "WIDTH " + std::to_string(tmp_int);
      tmp_string += '\n';
      outfile << tmp_string;
    }
    else if(line_count == 9)
    {
      std::string tmp_string = line.substr(7,5);
      // std::cout << tmp_string << std::endl;
      int tmp_int = std::stoi(tmp_string);
      tmp_int = tmp_int -1;
      tmp_string = "POINTS " + std::to_string(tmp_int); 
      tmp_string += '\n';
      outfile << tmp_string;
    }
    else
    {
      line += '\n';
      outfile << line;
    }
    line_count++;
  }

  fs::remove(input_file);
  fs::copy_file(tmp_file_path, input_file);
  fs::remove(tmp_file_path);
}

void pcd_to_ply(fs::path input_file, bool filter = false, int npoints = 4000, bool binary = true)
{
  std::cout << "Parsing file " << input_file.filename() << std::endl;
  //CLOUD
  PointCloud::Ptr pc (new PointCloud);

  //FILTER
  rand_filter.setSample((unsigned int) npoints);
  rand_filter.setSeed(std::rand());

  //DIRECTORIES
  std::string file_ext, in_filename, out_filename, out_filename_path;
  fs::path out_file_path;

  file_ext = input_file.extension();
  in_filename = input_file.stem();
  out_filename = in_filename + ".ply";
  out_file_path = ply_path / out_filename;

  //READ AND WRITE

  if (file_ext == ".pcd"){
    // if(pcd_reader.read(input_file.string(), *pc) != 0)
    // {
    //   while(pcd_reader.read(input_file.string(), *pc) != 0)
    //     read_txt(input_file);
    // }

    pcd_reader.read(input_file.string(), *pc);
    if (filter)
    {
      rand_filter.setInputCloud(pc);
      rand_filter.filter(*pc);
    }

    ply_writer.write(out_file_path.string(), *pc, binary, false);
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
  ply_path = current_path.parent_path() / "ply";

  if(!fs::exists(ply_path))
    fs::create_directory(ply_path);

  if(argc < 2)
    for(const auto &entry : fs::directory_iterator(current_path))
    {
      pcd_to_ply(entry.path(), true, 25000, true);
    }
  else
  {
    fs::path input_dir = argv[1];
    pcd_to_ply(input_dir, true, 25000, true);
  }

  return 0;
}

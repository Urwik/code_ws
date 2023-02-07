// cpp
#include <iostream>
#include <stdlib.h>
#include <filesystem>

// PCL
#include <pcl/common/common.h>
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

#include "tqdm.hpp"

namespace fs = std::filesystem;

//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

typedef pcl::PointXYZLNormal PointT;
typedef pcl::PointCloud<PointT> PointCloud;

pcl::PCDReader pcd_reader;
pcl::PLYWriter ply_writer;
pcl::RandomSample<PointT> rand_filter;
pcl::PassThrough<PointT> pass;
fs::path current_path;
fs::path ply_path;

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


void pcd_to_ply(PointCloud::Ptr &cloud, const std::string filename, bool binary = true)
{
  std::string out_filename;
  fs::path out_file_path;
  current_path = fs::current_path();
  ply_path = current_path.parent_path() / "ply_xyzlabelnormal";
  if(!fs::exists(ply_path))
  fs::create_directory(ply_path);

  out_filename = filename + ".ply";
  out_file_path = ply_path / out_filename;

  ply_writer.write(out_file_path.string(), *cloud, binary, false);
}


PointCloud::Ptr downSampleCloud(PointCloud::Ptr &cloud_in, const int target_points, const std::string filename)
{
  PointCloud::Ptr cloud_out (new PointCloud);
  PointCloud::Ptr cloud_floor (new PointCloud);
  PointCloud::Ptr cloud_structure (new PointCloud);

  int points_remove = cloud_in->points.size() - target_points;

  if(points_remove < 0)
  {
    std::cout << "\nThe number of target points is higher than the actual points "
    "in the cloud "<< filename <<", reduce the number of target points" << std::endl;
    // exit(EXIT_FAILURE);
  }
  else
  {
    pcl::Indices uselsess;
    pcl::removeNaNFromPointCloud<PointT>(*cloud_in, *cloud_in, uselsess);
    pcl::removeNaNNormalsFromPointCloud<PointT>(*cloud_in, *cloud_in, uselsess);

    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("label");
    pass.setFilterLimits(0, 0);
    pass.setNegative(false);
    pass.filter(*cloud_floor);
    pass.setNegative(true);
    pass.filter(*cloud_structure);

    pcl::RandomSample<PointT> rs;
    rs.setInputCloud(cloud_floor);
    rs.setSample((unsigned int) cloud_floor->points.size()- points_remove);
    rs.setSeed(std::rand());
    rs.filter(*cloud_floor);

    *cloud_out = *cloud_structure + *cloud_floor;
  }

  return cloud_out;
}


void read_file (fs::path input_file){
  PointCloud::Ptr pc (new PointCloud);
  pcl::io::loadPCDFile(input_file, *pc);

  std::cout << pc->width << std::endl;
  std::cout << pc->points.size() << std::endl;
}


int main(int argc, char **argv)
{
  fs::path current_dir = fs::current_path();

  PointCloud::Ptr cloud (new PointCloud);
  PointCloud::Ptr sampled_cloud (new PointCloud);
  std::string filename;

  bool DOWNSAMPLE = true;

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
      filename = entry.stem().string();
      cloud = readCloud(entry);
      if (DOWNSAMPLE)
      {
        sampled_cloud = downSampleCloud(cloud, 25000, filename);
        pcd_to_ply(sampled_cloud, filename);
      }
      else
        pcd_to_ply(cloud, filename);
    }
    std::cout << std::endl;
  }
  else
  {
    fs::path entry = argv[1];
    cloud = readCloud(entry);
    if (DOWNSAMPLE)
    {
      sampled_cloud = downSampleCloud(cloud, 25000, entry.stem().string());
      pcd_to_ply(sampled_cloud, entry.stem().string());
    }
    else
      pcd_to_ply(cloud, entry.stem().string());
  }

  return 0;
}

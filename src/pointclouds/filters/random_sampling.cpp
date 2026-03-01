// C++
#include <iostream>
#include <algorithm>
#include <filesystem>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/visualization/pcl_visualizer.h>
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
typedef pcl::PointXYZLNormal PointT;
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


PointCloud::Ptr randomSample(PointCloud::Ptr &cloud_in, const int target_points=25000)
{
  PointCloud::Ptr out_cloud (new PointCloud);
  pcl::RandomSample<PointT> rs;
  rs.setInputCloud(cloud_in);
  rs.setSample(target_points);
  rs.filter(*out_cloud);

  return out_cloud;
}


void writeCloud(PointCloud::Ptr &cloud_in, fs::path entry)
{
  pcl::PLYWriter ply_writer;
  
  fs::path abs_file_path = fs::current_path() / "ply_xyzln_sampled";
  if (!fs::exists(abs_file_path)) 
    fs::create_directory(abs_file_path);

  std::string filename = entry.stem().string() + ".ply";
  
  abs_file_path = abs_file_path / filename;
  std::cout << "Writing cloud to: " << abs_file_path.string() << std::endl;
  ply_writer.write(abs_file_path, *cloud_in, true);
}


int main(int argc, char **argv)
{
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS); //OCULTA TODOS LOS MENSAJES DE PCL

  PointCloud::Ptr cloud_in (new PointCloud);
  PointCloud::Ptr cloud_out (new PointCloud);

  fs::path current_dir = fs::current_path();

  const int TGT_POINTS = 20000;
  std::vector<std::string> clouds_ids{"00000","00004", "00147", "00360", "00524", "00227", "00345", "00181"};

  for (const std::string &cloud_id : tq::tqdm(clouds_ids))
  {
    fs::path cloud_path = current_dir / "ply_xyzln" / (cloud_id + ".ply");
    std::cout << "Processing cloud: " << cloud_path.string() << std::endl;
    cloud_in = readCloud(cloud_path);
    cloud_out = randomSample(cloud_in, TGT_POINTS);
    writeCloud(cloud_out, cloud_path);
  }


  // if(argc < 2)
  // {
  //   std::vector<fs::path> path_vector;
  //   for(const auto &entry : fs::directory_iterator(current_dir))
  //   {
  //     if(entry.path().extension() == ".pcd" || entry.path().extension() == ".ply")
  //       path_vector.push_back(entry.path());
  //   }

  //   for(const fs::path &entry : tq::tqdm(path_vector))
  //   {
  //     cloud_in = readCloud(entry);
  //     cloud_out = randomSample(cloud_in, TGT_POINTS);
  //     writeCloud(cloud_out, entry);

  //   }
  // }
  // else
  // {
  //   fs::path entry = argv[1];
  //   cloud_in = readCloud(entry);
  //   cloud_out = randomSample(cloud_in, TGT_POINTS);
  //   writeCloud(cloud_out, entry);
  // }

  std::cout << GREEN << "COMPLETED!!" << RESET << std::endl;
  return 0;
}




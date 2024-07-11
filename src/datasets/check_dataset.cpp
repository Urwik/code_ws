// C++
#include <iostream>
#include <stdlib.h>
#include <filesystem>
#include <algorithm>
#include <vector>
#include <math.h>

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
#include "../pointclouds/read_cloud.hpp"
#include "tqdm.hpp"

namespace fs = std::filesystem;

//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"  
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"

typedef pcl::PointXYZLNormal PointLN;
typedef pcl::PointCloud<PointLN> PointCloud;

pcl::PCDReader pcd_reader;
pcl::PLYReader ply_reader;
fs::path current_path;
std::vector<int> n_points;
std::vector<std::string> cloud_names;
std::vector<std::string> error_files;
std::vector<std::string> readed_files;
std::vector<std::string> duplicated_files;

template<typename T>
int check_num_points(typename pcl::PointCloud<T>::Ptr cloud_in, int target_points)
{
  // if (cloud_in->points.size() == 0){
  //   return 5;
  // }

  if (cloud_in->points.size() != target_points){
    return -1;
  }
  else
    return 0;
}


template<typename T>
int check_nan_points(typename pcl::PointCloud<T>::Ptr cloud_in)
{
  for (const T& point : cloud_in->points)
  {
    if(std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z) ||  std::isnan(point.label) || std::isnan(point.normal_x) || std::isnan(point.normal_y) || std::isnan(point.normal_z) || std::isnan(point.curvature))
    {
      return -1;
    }
  }
  return 0;
}


void check_filenames(fs::path current_path_)
{
  std::stringstream ss;

  std::cout << "Mising files:" << std::endl;
  for (int i = 0; i < 10000; i++)
  {
    ss.str("");
    ss << std::setfill('0') << std::setw(5)  << i << ".ply";
    fs::path file_path = current_path / ss.str();

    if(!fs::exists(file_path))
      std::cout << ss.str() << std::endl;
  }
}


int check_labels(fs::path input_file_)
{
  PointCloud::Ptr pc (new PointCloud);
  std::string file_ext = input_file_.extension();
  std::string filename = input_file_.filename();
  int correct = 0;


  if (file_ext == ".pcd")
    correct = pcd_reader.read(input_file_.string(), *pc);
  else if (file_ext == ".ply")
   correct =  ply_reader.read(input_file_.string(), *pc);
  else
    return -1;

  // CHECK IF ALL LABLES ARE A NUMBER
  for (auto& point : pc->points)
  {
    if(std::isnan(point.label))
    {
      std::cout << RED << "Wrong labels in: " << filename << RESET << std::endl;
      error_files.push_back(filename);
      return -1;
    }
  } 
    
  std::cout << GREEN << "Correct labels in: " << filename << RESET << std::endl;

  return 0;
}


void check_duplicated(fs::path input_file_)
{
  std::string filename = input_file_.filename();

  if (std::count(readed_files.begin(), readed_files.end(), filename)) 
  {
    duplicated_files.push_back(filename);
  }
}


int main(int argc, char **argv)
{
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);  
  pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZLNormal>);
  pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZLNormal>);

  fs::path current_dir = fs::current_path();
  std::string dataset_type = "ply_xyzln_fixedSize";

  std::vector<fs::path> cloud_paths;

  std::vector<fs::path> empty_clouds;
  
  if(argc < 2)
  {
    std::cout << "Getting cloud paths..." << std::endl;
    for(const auto &entry : fs::directory_iterator(current_dir))
    {
      if (entry.is_directory()){

        fs::path set_dir = entry.path() / dataset_type;
        std::cout << "Getting clouds from set: " << set_dir << std::endl;

        for(const auto &cloud_entry : fs::directory_iterator(set_dir)) {
          if(cloud_entry.path().extension().string() == ".pcd" || cloud_entry.path().extension().string() == ".ply"){
            cloud_paths.push_back(cloud_entry.path());
          }
        }
      }
    }
    
    // std::ifstream file("/home/arvc/workSpaces/code_ws/examples/pointclouds/empty_clouds.txt");
    // std::string line;

    // while (std::getline(file, line))
    // {
    //   cloud_paths.push_back(line);
    // }
    // file.close();

    std::cout << "Checking clouds..." << std::endl;
    int num_points_correct = 0;
    int nan_points_correct = 0;
    for(const fs::path &cloud_entry : cloud_paths)
    {
      cloud_in = readPointCloud<PointLN>(cloud_entry.string());

      num_points_correct = check_num_points<PointLN>(cloud_in, 20000);
      nan_points_correct = check_nan_points<PointLN>(cloud_in);

      // if (num_points_correct == -1)
      // {
      //   std::cout << RED << "Error in number of points: " << cloud_entry.string() << " | N_points: " << cloud_in->size() << RESET << std::endl;
      // }

      if (nan_points_correct == -1)
      {
        std::cout << RED << "Error in nan points: " << cloud_entry.string() << RESET << std::endl;
      }

      // if (num_points_correct == 5)
      // {
      //   std::cout << RED << "Empty cloud: " << cloud_entry.string() << RESET << std::endl;
      //   empty_clouds.push_back(cloud_entry);
      // }
    }
  }

  
  // fs::path empty_clouds_file_path = "/home/arvc/workSpaces/code_ws/examples/pointclouds/empty_clouds.txt";

  // std::cout << "Writing discarded clouds to " << empty_clouds_file_path << std::endl;

  // std::ofstream file(empty_clouds_file_path);
  // for (const fs::path &cloud_path : empty_clouds)
  // {
  //   file << cloud_path.string() << std::endl;
  // }
  // file.close();
  std::cout << "Number of clouds: " << cloud_paths.size() << std::endl;
  std::cout << "Completed check." << std::endl;

  return 0;
}
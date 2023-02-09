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

namespace fs = std::filesystem;

//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"  
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"

typedef pcl::PointXYZLNormal PointT;
typedef pcl::PointCloud<PointT> PointCloud;

pcl::PCDReader pcd_reader;
pcl::PLYReader ply_reader;
fs::path current_path;
std::vector<int> n_points;
std::vector<std::string> cloud_names;
std::vector<std::string> error_files;
std::vector<std::string> readed_files;
std::vector<std::string> duplicated_files;

int check_cloud(fs::path input_file)
{
  PointCloud::Ptr pc (new PointCloud);
  std::string file_ext = input_file.extension();
  int correct = 0;

  if (file_ext == ".pcd")
    correct = pcd_reader.read(input_file.string(), *pc);
  else if (file_ext == ".ply")
   correct =  ply_reader.read(input_file.string(), *pc);
  else
    return -1;


  if (correct == 0)
  {
    if (pc->points.size() != 25000)
      cloud_names.push_back(input_file.stem().string());

    n_points.push_back(pc->points.size());
  }
  else
    std::cout << "Error in Cloud: " << input_file.stem().string() << std::endl;

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
  current_path = fs::current_path();

  if(argc < 2)
  {
    // check_filenames(current_path);

    for(const auto &entry : fs::directory_iterator(current_path))
    {
      check_labels(entry.path());
      check_cloud(entry.path());
    }
  }
  else
  {
    fs::path input_dir = argv[1];
    check_labels(input_dir);
    check_cloud(input_dir);
  }

  int total = 0;

  for(int point : n_points)
    total += point;

  double mean = total / n_points.size();
  auto max = *std::max_element(n_points.begin(), n_points.end());
  auto min = *std::min_element(n_points.begin(), n_points.end());


  std::cout << "Max: " << max << std::endl;
  std::cout << "Min: " << min << std::endl;
  std::cout << "Mean: " << mean << std::endl;

  if(!error_files.empty())
  {
    std::cout << "Files with error in its labels: " << std::endl;
    for (std::string& file : error_files)
      std::cout << file << std::endl;      
  }

  if(!duplicated_files.empty())
  {
    std::cout << "Duplicated files: " << std::endl;
    for (std::string& file : duplicated_files)
      std::cout << file << std::endl;      
  }


  if(max != 25000 || min != 25000 || mean != 25000)
  {
    std::cout << "Dataset contains errors:" << std::endl;
    std::cout << "Clouds with wrong num of points" << std::endl;
    for(auto name : cloud_names)
      std::cout << name << std::endl;
  }
  else
    std::cout << "Dataset is correct" << std::endl;

  return 0;
}

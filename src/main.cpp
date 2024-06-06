#include "pointclouds/filters/random_downsample_cloud.hpp"
#include "pointclouds/read_cloud.hpp"
#include "pointclouds/write_cloud.hpp"
#include "pointclouds/features/normal_estimation.hpp"
#include "tqdm.hpp"

#include <iostream>
#include <filesystem>

#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>

#include <thread>
#include <vector>
#include <unordered_map> 

using namespace std;

namespace fs = std::filesystem;



// Function to find the mode of a vector 
int findMode(const vector<int>& nums) 
{ 
    // Create an unordered map to store frequency of each 
    // element 
    unordered_map<int, int> frequency; 
  
    // Iterate through the vector and update frequency of 
    // each element 
    for (int num : nums) { 
        frequency[num]++; 
    } 
  
    // Initialize variables to keep track of mode and its 
    // frequency 
    int mode = 0; 
    int maxFrequency = 0; 
  
    // Iterate through the unordered map and find the 
    // element with maximum frequency 
    for (const auto& pair : frequency) { 
        if (pair.second > maxFrequency) { 
            maxFrequency = pair.second; 
            mode = pair.first; 
        } 
    } 
  
    // Return the mode 
    return mode; 
}

// void writeCloud(PointCloud::Ptr &cloud_in, fs::path entry)
// {
//   pcl::PLYWriter ply_writer;
  
//   fs::path abs_file_path = fs::current_path().parent_path() / "ply_xyzl_fixedSize";
//   if (!fs::exists(abs_file_path)) 
//     fs::create_directory(abs_file_path);

//   std::string filename = entry.stem().string() + ".ply";
  
//   abs_file_path = abs_file_path / filename;
//   ply_writer.write(abs_file_path, *cloud_in, true);
// }

void writeDiscardedClouds(std::vector<std::string> &discarded_clouds)
{
  fs::path abs_file_path = fs::current_path().parent_path() / "discarded_clouds.txt";

  std::ofstream file(abs_file_path);
  for (const std::string &cloud : discarded_clouds)
  {
    file << cloud << std::endl;
  }
  file.close();
}

void node( fs::path entry, int target_points)
{
  PointCloudLN::Ptr cloud_in (new PointCloudLN);
  PointCloudLN::Ptr cloud_out (new PointCloudLN);

  cloud_in = readPointCloud<PointLN>(entry.string());
  // cloud_out = normalsByRadius(cloud_in, 0.05);
  cloud_out = randomSampleCloud(cloud_in, target_points);
  writeCloud<PointLN>(cloud_out, entry, "ply_xyzln_fixedSize");
}

int main(int argc, char** argv)
{

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);  
  PointCloud::Ptr cloud_in (new PointCloud);
  PointCloud::Ptr cloud_out (new PointCloud);

  fs::path current_dir = fs::current_path();

  std::vector<std::thread> threads;
  std::vector<int> cloud_sizes;
  std::vector<std::string> discarded_clouds;
  std::vector<fs::path> accepted_clouds;

  if(argc < 2)
  {
    std::vector<fs::path> path_vector;
    for(const auto &entry : fs::directory_iterator(current_dir))
    {
      if(entry.path().extension() == ".pcd" || entry.path().extension() == ".ply")
        path_vector.push_back(entry.path());


/*       if (entry.is_directory()){
        if (entry.path().stem() == "pcd") {
          for(const auto &sub_entry : fs::directory_iterator(entry)) {
            if(sub_entry.path().extension() == ".pcd")
              path_vector.push_back(sub_entry.path());
          }
        }
      } */
    }

    int target_points = 20000;

    std::cout << "Reading clouds..." << std::endl;
    for(const fs::path &entry : tq::tqdm(path_vector))
    {
      cloud_in = readPointCloud<PointLN>(entry.string());
      int tmp_cloud_size = cloud_in->points.size();

      if (tmp_cloud_size < target_points)
        discarded_clouds.push_back(entry.stem());
      else
        accepted_clouds.push_back(entry);

      if (tmp_cloud_size < target_points)
        std::cout << "Cloud " << entry.stem() << " has " << tmp_cloud_size << " points" << std::endl;      
    }

    // writeDiscardedClouds(discarded_clouds);


    // std::cout << "Downsampling clouds..." << std::endl;

    // for(const fs::path &entry : tq::tqdm(accepted_clouds))
    //   threads.push_back(std::thread(node, entry, target_points));


    // for(auto &t : threads)
    //   t.join();
  }

  else
  {
    fs::path entry = argv[1];
    node(entry, 20000);
  }



  std::cout  << "COMPLETED!!" << std::endl;
  return 0;
}
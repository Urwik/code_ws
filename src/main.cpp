#include "pointclouds/filters/random_downsample_cloud.hpp"
#include "pointclouds/read_cloud.hpp"
#include "pointclouds/write_cloud.hpp"
#include "pointclouds/features/normal_estimation.hpp"
#include "pointclouds/filters/farthest_point_downsample.hpp"
#include "pointclouds/visualization/view_cloud_normals.hpp"
#include "tqdm.hpp"

#include <iostream>
#include <filesystem>
#include <time.h>

#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>

#include <thread>
#include <vector>
#include <unordered_map> 

using namespace std;

namespace fs = std::filesystem;

bool hasNaN(const pcl::PointCloud<PointLN>::ConstPtr& cloud) {

  bool nan = false;
  for (const auto& point : cloud->points) {
      if (std::isnan(point.x)){
        std::cout <<  "x is nan" << std::endl;
        nan = true;
        continue;
      }
      if (std::isnan(point.y)){
        std::cout <<  "y is nan" << std::endl;
        nan = true;
        continue;
      }
      if (std::isnan(point.z)){
        std::cout <<  "z is nan" << std::endl;
        nan = true;
        continue;
      }
      if (std::isnan(point.normal_x)){
        std::cout <<  "normal_x is nan" << std::endl;
        nan = true;
        continue;
      }
      if (std::isnan(point.normal_y)){
        std::cout <<  "normal_y is nan" << std::endl;
        nan = true;
        continue;
      }
      if (std::isnan(point.normal_z)){
        std::cout <<  "normal_z is nan" << std::endl;
        nan = true;
        continue;
      }
      if (std::isnan(point.label)){
        std::cout <<  "label is nan" << std::endl;
        nan = true;
        continue;
      }
      if (std::isnan(point.curvature)){
        std::cout <<  "curvature is nan" << std::endl;
        nan = true;
        continue;
      }
  }
  return nan;
}

void writeDiscardedClouds(std::vector<fs::path> &discarded_clouds)
{
  fs::path abs_file_path = fs::current_path() / "discarded_clouds.txt";

  std::cout << "Writing discarded clouds to " << abs_file_path << std::endl;

  std::ofstream file(abs_file_path);
  for (const fs::path &cloud_path : discarded_clouds)
  {
    file << cloud_path.string() << std::endl;
  }
  file.close();
}

std::vector<fs::path> nan_clouds;

void node( fs::path entry, int target_points)
{
  PointCloudLN::Ptr cloud_in (new PointCloudLN);
  PointCloudLN::Ptr cloud_out (new PointCloudLN);

  std::cout << "Reading cloud " << entry.string() << std::endl;
  cloud_in = readPointCloud<PointLN>(entry.string());
  // cloud_out = normalsByRadius<PointLN>(cloud_in, 0.05, true);
  // cloud_out = normalsByNeighbours<PointLN>(cloud_in, 30, true);
  cloud_out = randomSampleCloud(cloud_in, target_points);
  // cloud_out = farthestPointDownSample<PointLN>(cloud_in, target_points);


  if (cloud_out->points.size() == 0) {
    std::cout << "Cloud " << entry.string() << " is empty after random sample " << std::endl;

    fs::path abs_file_path = fs::current_path() / "discarded_clouds.txt";
    std::ofstream file(abs_file_path);
    file << entry.string() << std::endl;
    file.close();
  }
  else
  {
    // rewriteCloud<PointLN>(cloud_out, entry);
    writeCloud<PointLN>(cloud_out, entry, "ply_xyzln_fixedSize");
  }

}

int main(int argc, char** argv)
{

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);  
  PointCloudLN::Ptr cloud_in (new PointCloudLN);
  PointCloudLN::Ptr cloud_out (new PointCloudLN);

  fs::path current_dir = fs::current_path();

  std::vector<std::thread> threads;
  std::vector<int> cloud_sizes;
  std::vector<fs::path> discarded_clouds;
  std::vector<fs::path> accepted_clouds;


//--------------------------------------------------------------------------------//
// FOR ALL CLOUDS IN CURRENT DIRECTORY                                            //
//--------------------------------------------------------------------------------//
  if(argc < 2)
  {
    std::vector<fs::path> cloud_paths;
    for(const auto &entry : fs::directory_iterator(current_dir))
    {
      // if(entry.path().extension() == ".pcd" || entry.path().extension() == ".ply")
      //   path_vector.push_back(entry.path());


      // Entirely retTruss dataset
      if (entry.is_directory()){

        fs::path set_dir = entry.path() / "ply_xyzln_fixedSize";
        std::cout << "Getting clouds from set: " << set_dir << std::endl;

        for(const auto &cloud_entry : fs::directory_iterator(set_dir)) {
          if(cloud_entry.path().extension().string() == ".pcd" || cloud_entry.path().extension().string() == ".ply"){
            cloud_paths.push_back(cloud_entry.path());
          }
        }
      }
    }
    
    std::cout << "Initial number of clouds: " << cloud_paths.size() << std::endl;
    int target_points = 20000;

    // std::cout << "Reading clouds..." << std::endl;
    // for(const fs::path &cloud_entry : tq::tqdm(cloud_paths))
    // {
    //   cloud_in = readPointCloud<PointLN>(cloud_entry.string());

    //   // pcl::Indices uselsess;
    //   // int initial_size = cloud_in->points.size();
    //   // pcl::removeNaNFromPointCloud<PointLN>(*cloud_in, *cloud_in, uselsess);
    //   // pcl::removeNaNNormalsFromPointCloud<PointLN>(*cloud_in, *cloud_in, uselsess);
    //   // int final_size = cloud_in->points.size();
    //   // if (initial_size != final_size)
    //   //   std::cout << "Cloud " << cloud_entry.string() << " has NaN values. Discarding..." << std::endl;
    //   // rewriteCloud<PointLN>(cloud_in, cloud_entry);


    //   int tmp_cloud_size = cloud_in->points.size();

    //   if (tmp_cloud_size < target_points) {
    //     std::cout << std::endl << "Cloud " << cloud_entry.string() << " has less than " << target_points << " points. Discarding..." << std::endl;
    //     discarded_clouds.push_back(cloud_entry);
    //   }
    //   else
    //   {
    //     accepted_clouds.push_back(cloud_entry);
    //   }
    // }

    // writeDiscardedClouds(discarded_clouds);


    // std::cout << "Downsampling clouds..." << std::endl;
    // for(const fs::path &entry : tq::tqdm(accepted_clouds))
    //   threads.push_back(std::thread(node, entry, target_points));

    // std::cout << "Estimating normals..." << std::endl;
    // for (const fs::path &entry : tq::tqdm(cloud_paths))
    //   node(entry, 20000);


    // std::cout << "Downsampling clouds..." << std::endl;
    // for(const fs::path &entry : tq::tqdm(cloud_paths))
    //   threads.push_back(std::thread(node, entry, 20000));

    // for(auto &t : threads)
    //   t.join();

    // writeDiscardedClouds(nan_clouds);
  }

//--------------------------------------------------------------------------------//
// ONLY ONE CLOUD                                                                 //
//--------------------------------------------------------------------------------//
  else {
    
    std::cout << "Reading one cloud..." << std::endl;
    fs::path entry = argv[1];

    node(entry, 20000);
  
  }
      

  std::cout  << "COMPLETED!!" << std::endl;
  return 0;
}


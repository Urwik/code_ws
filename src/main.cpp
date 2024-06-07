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
        }
        if (std::isnan(point.y)){
            std::cout <<  "y is nan" << std::endl;
            nan = true;
        }
        if (std::isnan(point.z)){
            std::cout <<  "z is nan" << std::endl;
            nan = true;
        }
        if (std::isnan(point.normal_x)){
            std::cout <<  "normal_x is nan" << std::endl;
            nan = true;
        }
        if (std::isnan(point.normal_y)){
            std::cout <<  "normal_y is nan" << std::endl;
            nan = true;
        }
        if (std::isnan(point.normal_z)){
            std::cout <<  "normal_z is nan" << std::endl;
            nan = true;
        }
        if (std::isnan(point.label)){
            std::cout <<  "label is nan" << std::endl;
            nan = true;
        }
        if (std::isnan(point.curvature)){
            std::cout <<  "curvature is nan" << std::endl;
            nan = true;
        }
    }
    return nan;
}

void writeDiscardedClouds(std::vector<fs::path> &discarded_clouds)
{
  fs::path abs_file_path = fs::current_path().parent_path() / "discarded_clouds.txt";

  std::ofstream file(abs_file_path);
  for (const fs::path &cloud_path : discarded_clouds)
  {
    file << cloud_path.stem().string() << std::endl;
  }
  file.close();
}


void node( fs::path entry, int target_points)
{
  PointCloudLN::Ptr cloud_in (new PointCloudLN);
  PointCloudLN::Ptr cloud_out (new PointCloudLN);

  cloud_in = readPointCloud<PointLN>(entry.string());
  // cloud_out = normalsByRadius<PointLN>(cloud_in, 0.05);
  cloud_out = normalsByNeighbours<PointLN>(cloud_in, 30);
  // cloud_out = randomSampleCloud(cloud_in, target_points);
  // cloud_out = farthestPointDownSample<PointLN>(cloud_in, target_points);
  
  writeCloud<PointLN>(cloud_out, entry, "ply_xyzln_byNeighbours_30");
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

        fs::path set_dir = entry.path() / "ply_xyzln_byNeighbours_30";

        for(const auto &cloud_entry : fs::directory_iterator(set_dir)) {
          if(cloud_entry.path().extension().string() == ".pcd" || cloud_entry.path().extension().string() == ".ply"){
            cloud_paths.push_back(cloud_entry.path());
          }
        }
      }
    }
    

    int target_points = 20000;

    std::cout << "Reading clouds..." << std::endl;
    for(const fs::path &cloud_entry : tq::tqdm(cloud_paths))
    {
      cloud_in = readPointCloud<PointLN>(cloud_entry.string());

      plotCloudWithNormals<PointLN>(cloud_in);

      // if (hasNaN(cloud_in)){
      //   std::cout << "Cloud " << cloud_entry.string() << " has NaN values. Discarding..." << std::endl;
      // }


      // int tmp_cloud_size = cloud_in->points.size();

      // if (tmp_cloud_size < target_points)
      //   discarded_clouds.push_back(cloud_entry);
      // else
      //   accepted_clouds.push_back(cloud_entry);

    }
    return 0;

    writeDiscardedClouds(discarded_clouds);


    std::cout << "Downsampling clouds..." << std::endl;
    for(const fs::path &entry : tq::tqdm(accepted_clouds))
      threads.push_back(std::thread(node, entry, target_points));


    for(auto &t : threads)
      t.join();
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


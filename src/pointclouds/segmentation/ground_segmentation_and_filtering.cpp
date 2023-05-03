// C++
#include <iostream>
#include <algorithm>
#include <filesystem>
#include <chrono>

#include "arvc_utils.hpp"
#include "tqdm.hpp"
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

namespace fs = std::filesystem;
using namespace std;

class remove_ground
{
private:
  /* data */
public:
  fs::path path;
  PointCloud::Ptr cloud_in;
  PointCloud::Ptr cloud_out;
  bool visualize;

  remove_ground(fs::path _path)
  {
    this->path = _path;
    this->cloud_in = PointCloud::Ptr (new PointCloud);
    this->cloud_out = PointCloud::Ptr (new PointCloud);
    this->visualize = false;
  }

  ~remove_ground()
  {
    this->path.clear();
    this->cloud_in->clear();
    this->cloud_out->clear();
    this->visualize = false;
  }

  int
  run()
  {
    PointCloudI::Ptr cloud_in_intensity (new PointCloudI);
    PointCloud::Ptr cloud_in_xyz (new PointCloud);
    PointCloud::Ptr cloud_out_xyz (new PointCloud);
    PointCloud::Ptr tmp_cloud (new PointCloud);

    pcl::IndicesPtr current_indices (new pcl::Indices);
    pcl::ModelCoefficientsPtr tmp_plane_coefss (new pcl::ModelCoefficients);

    gt_indices ground_truth_indices;

    vector<pcl::PointIndices> regrow_clusters;
    vector<int> valid_clusters;

    cloud_in_intensity = arvc::readCloud(this->path);
    ground_truth_indices = arvc::getGroundTruthIndices(cloud_in_intensity);
    cloud_in_xyz = arvc::parseToXYZ(cloud_in_intensity);
    *this->cloud_in = *cloud_in_xyz;

    // This is temporal, only for get the correct biggest plane
    tmp_cloud = arvc::voxel_filter(cloud_in_xyz, 0.05f);
    (this->visualize) ? arvc::visualizeCloud(tmp_cloud) : void();
    tmp_plane_coefss = arvc::compute_planar_ransac(tmp_cloud, true ,0.15f, 1000);
    current_indices = arvc::get_points_near_plane(cloud_in_xyz, tmp_plane_coefss, 0.15f);

    // Filter points with similar curvature lying in same plane
    tmp_cloud = arvc::extract_indices(cloud_in_xyz, current_indices, false);
    cloud_out_xyz = arvc::extract_indices(cloud_in_xyz, current_indices, true);

    regrow_clusters = arvc::regrow_segmentation(tmp_cloud);
    valid_clusters = arvc::validate_clusters(tmp_cloud, regrow_clusters);

    pcl::IndicesPtr tmp_indices (new pcl::Indices);
    PointCloud::Ptr tmp_cloud_2 (new PointCloud); 
    for(int clus_indx : valid_clusters)
    {
      *tmp_indices = regrow_clusters[clus_indx].indices;
      tmp_cloud_2 = arvc::extract_indices(tmp_cloud, tmp_indices);
      *cloud_out_xyz += *tmp_cloud_2;
    }

    cloud_out_xyz = arvc::radius_outlier_removal(cloud_out_xyz, 0.05f, 5);
    (this->visualize) ? arvc::visualizeCloud(cloud_out_xyz) : void();
    *this->cloud_out = *cloud_out_xyz;

    return 0;
  }
};


////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  std::cout << YELLOW << "Running your code..." << RESET << std::endl;
  auto start = std::chrono::high_resolution_clock::now();


  // EVERY CLOUD IN THE CURRENT FOLDER
  if(argc < 2)
  {
    fs::path current_dir = fs::current_path();
    std::vector<fs::path> path_vector;
    for(const auto &entry : fs::directory_iterator(current_dir))
    {
      if(entry.path().extension() == ".pcd" || entry.path().extension() == ".ply")
        path_vector.push_back(entry.path());
    }

    for(const fs::path &entry : tq::tqdm(path_vector))
    {
    }


  }


  // ONLY ONE CLOUD PASSED AS ARGUMENT IN CURRENT FOLDER
  else
  {
    // Get the input data
    fs::path entry = argv[1];
  
    remove_ground rg(entry);
    rg.visualize = true;
    rg.run();



  }


  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

  std::cout << "Computation Time: " << duration.count() << " ms" << std::endl;
  std::cout << YELLOW << "Code end!!" << RESET << std::endl;
  return 0;
}
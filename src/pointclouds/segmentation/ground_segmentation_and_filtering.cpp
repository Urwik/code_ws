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
  pcl::IndicesPtr truss_idx;
  pcl::IndicesPtr ground_idx;
  pcl::IndicesPtr gt_truss_idx;
  pcl::IndicesPtr gt_ground_idx;
  pcl::IndicesPtr low_density_idx;

  bool visualize;

  remove_ground(fs::path _path)
  {
    this->path = _path;
    this->cloud_in = PointCloud::Ptr (new PointCloud);
    this->cloud_out = PointCloud::Ptr (new PointCloud);
    this->truss_idx = pcl::IndicesPtr (new pcl::Indices);
    this->ground_idx = pcl::IndicesPtr (new pcl::Indices);
    this->gt_truss_idx = pcl::IndicesPtr (new pcl::Indices);
    this->gt_ground_idx = pcl::IndicesPtr (new pcl::Indices);
    this->low_density_idx = pcl::IndicesPtr (new pcl::Indices);

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
    PointCloud::Ptr uncluster_cloud (new PointCloud);

    pcl::IndicesPtr coarse_ground_indices (new pcl::Indices);
    pcl::IndicesPtr coarse_truss_indices (new pcl::Indices);
    pcl::ModelCoefficientsPtr tmp_plane_coefss (new pcl::ModelCoefficients);

    gt_indices ground_truth_indices;

    vector<pcl::PointIndices> regrow_clusters;
    vector<int> valid_clusters;

    cloud_in_intensity = arvc::readCloud(this->path);
    ground_truth_indices = arvc::getGroundTruthIndices(cloud_in_intensity);
    *this->gt_ground_idx = ground_truth_indices.ground;
    *this->gt_truss_idx = ground_truth_indices.truss;
    cloud_in_xyz = arvc::parseToXYZ(cloud_in_intensity);
    *this->cloud_in = *cloud_in_xyz;

    // This is temporal, only for get the correct biggest plane
    tmp_cloud = arvc::voxel_filter(cloud_in_xyz, 0.05f);
    tmp_plane_coefss = arvc::compute_planar_ransac(tmp_cloud, true ,0.5f, 1000);
    coarse_ground_indices = arvc::get_points_near_plane(cloud_in_xyz, tmp_plane_coefss, 0.5f);
    coarse_truss_indices = arvc::inverseIndices(this->cloud_in, coarse_ground_indices);
    // coarse_truss_indices = arvc::ownInverseIndices(this->cloud_in, coarse_ground_indices);

    
    // Filter points with similar curvature lying in same plane
    tmp_cloud = arvc::extract_indices(cloud_in_xyz, coarse_ground_indices, false);
    cloud_out_xyz = arvc::extract_indices(cloud_in_xyz, coarse_truss_indices, false);
    regrow_clusters = arvc::regrow_segmentation(this->cloud_in, coarse_ground_indices);
    valid_clusters = arvc::validate_clusters(tmp_cloud, regrow_clusters, 0.2f);

    // Get unclustered_cloud
    // uncluster_cloud = arvc::extract_indices(tmp_cloud, regrow_clusters, true);
    // uncluster_cloud = arvc::radius_outlier_removal(uncluster_cloud, 0.1f, 5);
    // arvc::visualizeCloud(uncluster_cloud);

    // pcl::IndicesPtr tmp_indices (new pcl::Indices);
    // PointCloud::Ptr tmp_cloud_2 (new PointCloud); 
    // for(int clus_indx : valid_clusters)
    // {
    //   *tmp_indices = regrow_clusters[clus_indx].indices;
    //   tmp_cloud_2 = arvc::extract_indices(tmp_cloud, tmp_indices);
    //   *cloud_out_xyz += *tmp_cloud_2;
    // }


    for(int clus_indx : valid_clusters)
      coarse_truss_indices->insert(coarse_truss_indices->end(), regrow_clusters[clus_indx].indices.begin(), regrow_clusters[clus_indx].indices.end());

    *this->truss_idx = *coarse_truss_indices;
    *this->ground_idx = *arvc::inverseIndices(this->cloud_in, coarse_truss_indices);
    // *this->ground_idx = *arvc::ownInverseIndices(this->cloud_in, coarse_truss_indices);

    this->cloud_out = arvc::extract_indices(cloud_in_xyz, coarse_truss_indices, false);
    this->truss_idx = arvc::radius_outlier_removal(cloud_in_xyz, this->truss_idx, 0.1f, 5, false);
    this->ground_idx = arvc::inverseIndices(this->cloud_in, this->truss_idx);
    // this->ground_idx = arvc::ownInverseIndices(this->cloud_in, this->truss_idx);


    this->cloud_out = arvc::extract_indices(cloud_in_xyz, this->truss_idx, false);

    (this->visualize) ?  arvc::visualizeClouds(this->cloud_in, this->cloud_out) : void();


    // Compute metrics
    conf_matrix cm;
    cm = arvc::computeConfusionMatrix(this->gt_truss_idx, this->gt_ground_idx, this->truss_idx, this->ground_idx);

    metrics my_metrics;
    my_metrics = arvc::computeMetrics(cm);

    cout << "Confusion Matrix: " << endl;
    cout << "\tTP: " << cm.TP << endl;
    cout << "\tTN: " << cm.TN << endl;
    cout << "\tFP: " << cm.FP << endl;
    cout << "\tFN: " << cm.FN << endl;

    cout << "Metrics: " << endl;
    cout << "\tAccuracy: " << my_metrics.accuracy << endl;
    cout << "\tPrecision: " << my_metrics.precision << endl;
    cout << "\tRecall: " << my_metrics.recall << endl;
    cout << "\tF1_Score: " << my_metrics.f1_score << endl;


    return 0;
  }
};


class segment_planes
{
private:

public:
  fs::path path;
  PointCloud::Ptr cloud_in;
  PointCloud::Ptr cloud_out;
  bool visualize;

  // Constructor
  segment_planes(fs::path _path){
    this->path = _path;
    this->cloud_in = PointCloud::Ptr (new PointCloud);
    this->cloud_out = PointCloud::Ptr (new PointCloud);
    this->visualize = false;
  }

  // Destructor
  ~segment_planes(){
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

    regrow_clusters = arvc::regrow_segmentation(tmp_cloud, current_indices);
    valid_clusters = arvc::validate_clusters(tmp_cloud, regrow_clusters, 0.3f);

    pcl::IndicesPtr tmp_indices (new pcl::Indices);
    PointCloud::Ptr tmp_cloud_2 (new PointCloud); 
    for(int clus_indx : valid_clusters)
    {
      *tmp_indices = regrow_clusters[clus_indx].indices;
      tmp_cloud_2 = arvc::extract_indices(tmp_cloud, tmp_indices);
      *cloud_out_xyz += *tmp_cloud_2;
    }

    // cloud_out_xyz = arvc::radius_outlier_removal(cloud_out_xyz, 0.05f, 5);
    (this->visualize) ? arvc::visualizeCloud(cloud_out_xyz) : void();
    *this->cloud_out = *cloud_out_xyz;

    return 0;
  }
};


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

  std::cout << YELLOW << "Code end!!" << RESET << std::endl;
  std::cout << "Computation Time: " << duration.count() << " ms" << std::endl;
  return 0;
}
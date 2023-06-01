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
  pcl::IndicesPtr wrong_idx;
  pcl::IndicesPtr tp_idx;
  pcl::IndicesPtr fp_idx;
  pcl::IndicesPtr fn_idx;
  pcl::IndicesPtr tn_idx;

  metrics metricas;
  conf_matrix cm;

  bool visualize;
  bool compute_metrics;

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
    this->wrong_idx = pcl::IndicesPtr (new pcl::Indices);
    this->tp_idx = pcl::IndicesPtr (new pcl::Indices);
    this->fp_idx = pcl::IndicesPtr (new pcl::Indices);
    this->fn_idx = pcl::IndicesPtr (new pcl::Indices);
    this->tn_idx = pcl::IndicesPtr (new pcl::Indices);

    this->visualize = false;
    this->compute_metrics = false;
  }

  ~remove_ground()
  {
    this->path.clear();
    this->cloud_in->clear();
    this->cloud_out->clear();
    this->visualize = false;
  }

  void
  getConfMatrixIndexes()
  {
    for (size_t i = 0; i < this->truss_idx->size(); i++)
    {
      if(std::find(this->gt_truss_idx->begin(), this->gt_truss_idx->end(), this->truss_idx->at(i)) != this->gt_truss_idx->end())
        this->tp_idx->push_back(this->truss_idx->at(i));
      else
        this->fp_idx->push_back(this->truss_idx->at(i));
    }

    for (size_t i = 0; i < this->ground_idx->size(); i++)
    {
      if(std::find(this->gt_ground_idx->begin(), this->gt_ground_idx->end(), this->ground_idx->at(i)) != this->gt_ground_idx->end())
        this->tn_idx->push_back(this->ground_idx->at(i));
      else
        this->fn_idx->push_back(this->ground_idx->at(i));
    }
    
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
    ground_truth_indices.ground = pcl::IndicesPtr (new pcl::Indices);
    ground_truth_indices.truss = pcl::IndicesPtr (new pcl::Indices);

    vector<pcl::PointIndices> regrow_clusters;
    vector<int> valid_clusters;



    // Read pointcloud
    cloud_in_intensity = arvc::readCloudWithIntensity(this->path);
    ground_truth_indices = arvc::getGroundTruthIndices(cloud_in_intensity);
    *this->gt_ground_idx = *ground_truth_indices.ground;
    *this->gt_truss_idx = *ground_truth_indices.truss;
    this->cloud_in = arvc::parseToXYZ(cloud_in_intensity);



    // EXTRACT BIGGEST PLANE
    // This is temporal, only for get the correct biggest plane
    tmp_cloud = arvc::voxel_filter(this->cloud_in, 0.05f);
    tmp_plane_coefss = arvc::compute_planar_ransac(tmp_cloud, true ,0.5f, 1000);
    auto coarse_indices = arvc::get_points_near_plane(this->cloud_in, tmp_plane_coefss, 0.5f);
    coarse_ground_indices = coarse_indices.first;
    coarse_truss_indices = coarse_indices.second;

    PointCloud::Ptr coarse_ground_cloud (new PointCloud);
    PointCloud::Ptr coarse_truss_cloud (new PointCloud);
    coarse_ground_cloud = arvc::extract_indices(this->cloud_in, coarse_ground_indices, false);
    // (this->visualize) ? arvc::visualizeCloud(coarse_ground_cloud, 0, 0, 170) : void();
    coarse_truss_cloud = arvc::extract_indices(this->cloud_in, coarse_truss_indices, false);
    // (this->visualize) ? arvc::visualizeCloud(coarse_truss_cloud, 0, 0, 170) : void();
    arvc::visualizeClouds(coarse_truss_cloud, 0, 0, 170, coarse_ground_cloud, 0, 0, 170);
    

    // FILTER CLUSTERS BY EIGEN VALUES
    // cloud_out_xyz = arvc::extract_indices(this->cloud_in, coarse_truss_indices, false);
    regrow_clusters = arvc::regrow_segmentation(this->cloud_in, coarse_ground_indices);
    // valid_clusters = arvc::validate_clusters_by_ratio(this->cloud_in, regrow_clusters, 0.15f);
    // valid_clusters = arvc::validate_clusters_by_module(this->cloud_in, regrow_clusters, 1000.0f);
    valid_clusters = arvc::validate_clusters_hybrid(this->cloud_in, regrow_clusters, 0.3f, 1000.0f);

    for(int clus_indx : valid_clusters)
      coarse_truss_indices->insert(coarse_truss_indices->end(), regrow_clusters[clus_indx].indices.begin(), regrow_clusters[clus_indx].indices.end());

    *this->truss_idx = *coarse_truss_indices;
    *this->ground_idx = *arvc::inverseIndices(this->cloud_in, this->truss_idx);
    // *this->ground_idx = *arvc::ownInverseIndices(this->cloud_in, coarse_truss_indices);



    // FILTER CLOUD BY DENISTY
    this->truss_idx = arvc::radius_outlier_removal(this->cloud_in, this->truss_idx, 0.1f, 5, false);




    this->ground_idx = arvc::inverseIndices(this->cloud_in, this->truss_idx);
    // this->ground_idx = arvc::ownInverseIndices(this->cloud_in, this->truss_idx);
    this->cloud_out = arvc::extract_indices(this->cloud_in, this->truss_idx, false);
    tmp_cloud = arvc::extract_indices(this->cloud_in, this->ground_idx, false);
    (this->visualize) ?  arvc::visualizeClouds(this->cloud_out,0,0,170, tmp_cloud,0,0,170) : void();


    this->getConfMatrixIndexes();

    PointCloud::Ptr error_cloud (new PointCloud);
    PointCloud::Ptr truss_cloud (new PointCloud);
    PointCloud::Ptr ground_cloud (new PointCloud);
    pcl::IndicesPtr error_idx (new pcl::Indices);

    error_idx->insert(error_idx->end(), this->fp_idx->begin(), this->fp_idx->end());
    error_idx->insert(error_idx->end(), this->fn_idx->begin(), this->fn_idx->end());

    truss_cloud = arvc::extract_indices(cloud_in, this->tp_idx, false);
    ground_cloud = arvc::extract_indices(cloud_in,this->tn_idx, false);
    error_cloud = arvc::extract_indices(cloud_in, error_idx, false);

    pcl::visualization::PCLVisualizer my_vis;
    my_vis.setBackgroundColor(1,1,1);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> truss_color (truss_cloud, 0,255,0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> ground_color (ground_cloud, 100,100,100);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> error_color (error_cloud, 255,0,0);

    my_vis.addPointCloud(truss_cloud, truss_color, "truss_cloud");
    my_vis.addPointCloud(ground_cloud, ground_color, "wrong_cloud");
    my_vis.addPointCloud(error_cloud, error_color, "error_cloud");

    while (!my_vis.wasStopped())
    {
      my_vis.spinOnce(100);
    }
  


    (this->visualize) ?  arvc::visualizeClouds(this->cloud_in,0,0,170, this->cloud_out,0,0,170) : void();


    // Compute metrics
    if(this->compute_metrics)
    {
      this->cm = arvc::computeConfusionMatrix(this->gt_truss_idx, this->gt_ground_idx, this->truss_idx, this->ground_idx);
      this->metricas = arvc::computeMetrics(cm);
    }
    // cout << "Confusion Matrix: " << endl;
    // cout << "\tTP: " << cm.TP << endl;
    // cout << "\tTN: " << cm.TN << endl;
    // cout << "\tFP: " << cm.FP << endl;
    // cout << "\tFN: " << cm.FN << endl;

    // cout << "Metrics: " << endl;
    // cout << "\tAccuracy: " << my_metrics.accuracy << endl;
    // cout << "\tPrecision: " << my_metrics.precision << endl;
    // cout << "\tRecall: " << my_metrics.recall << endl;
    // cout << "\tF1_Score: " << my_metrics.f1_score << endl;


    return 0;
  }

  int 
  run2(){
    PointCloudI::Ptr cloud_in_intensity (new PointCloudI);
    PointCloud::Ptr cloud_in_xyz (new PointCloud);
    PointCloud::Ptr cloud_out_xyz (new PointCloud);
    PointCloud::Ptr tmp_cloud (new PointCloud);
    PointCloud::Ptr uncluster_cloud (new PointCloud);

    pcl::IndicesPtr coarse_ground_indices (new pcl::Indices);
    pcl::IndicesPtr coarse_truss_indices (new pcl::Indices);
    pcl::IndicesPtr idx_removed_by_density (new pcl::Indices);
    pcl::IndicesPtr idx_passed_density (new pcl::Indices);


    pcl::ModelCoefficientsPtr tmp_plane_coefss (new pcl::ModelCoefficients);

    gt_indices ground_truth_indices;
    ground_truth_indices.ground = pcl::IndicesPtr (new pcl::Indices);
    ground_truth_indices.truss = pcl::IndicesPtr (new pcl::Indices);

    vector<pcl::PointIndices> regrow_clusters;
    vector<int> valid_clusters;



    // Read pointcloud
    cloud_in_intensity = arvc::readCloudWithIntensity(this->path);
    ground_truth_indices = arvc::getGroundTruthIndices(cloud_in_intensity);
    *this->gt_ground_idx = *ground_truth_indices.ground;
    *this->gt_truss_idx = *ground_truth_indices.truss;

    // FILTER CLOUD BY DENISTY
    pcl::RadiusOutlierRemoval<PointI> radius_removal;
    radius_removal.setInputCloud(cloud_in_intensity);
    radius_removal.setRadiusSearch(0.1f);
    radius_removal.setMinNeighborsInRadius(5);
    radius_removal.setNegative(false);
    radius_removal.filter(*idx_passed_density);
    radius_removal.setNegative(true);
    radius_removal.filter(*idx_removed_by_density);
 
    this->cloud_in = arvc::parseToXYZ(cloud_in_intensity);


    // EXTRACT BIGGEST PLANE
    // This is temporal, only for get the correct biggest plane
    tmp_cloud = arvc::voxel_filter(this->cloud_in, idx_passed_density, 0.05f);
    tmp_plane_coefss = arvc::compute_planar_ransac(tmp_cloud, true ,0.2f, 1000);
    auto coarse_indices = arvc::get_points_near_plane(this->cloud_in, idx_passed_density, tmp_plane_coefss, 0.2f);
    coarse_ground_indices = coarse_indices.first;
    coarse_truss_indices = coarse_indices.second;

    PointCloud::Ptr coarse_ground_cloud (new PointCloud);
    PointCloud::Ptr coarse_truss_cloud (new PointCloud);
    coarse_ground_cloud = arvc::extract_indices(this->cloud_in, coarse_ground_indices, false);
    coarse_truss_cloud = arvc::extract_indices(this->cloud_in, coarse_truss_indices, false);
    
    cout << "Coarse truss vs Coarse Ground" << endl;
    arvc::visualizeClouds(coarse_truss_cloud, 0, 0, 170, coarse_ground_cloud, 0, 0, 170);
    

    // FILTER CLUSTERS BY EIGEN VALUES
    // cloud_out_xyz = arvc::extract_indices(this->cloud_in, coarse_truss_indices, false);
    regrow_clusters = arvc::regrow_segmentation(this->cloud_in, coarse_ground_indices);
    // valid_clusters = arvc::validate_clusters_by_ratio(this->cloud_in, regrow_clusters, 0.15f);
    // valid_clusters = arvc::validate_clusters_by_module(this->cloud_in, regrow_clusters, 1000.0f);
    valid_clusters = arvc::validate_clusters_hybrid(this->cloud_in, regrow_clusters, 0.3f, 1000.0f);

    for(int clus_indx : valid_clusters)
      coarse_truss_indices->insert(coarse_truss_indices->end(), regrow_clusters[clus_indx].indices.begin(), regrow_clusters[clus_indx].indices.end());

    *this->truss_idx = *coarse_truss_indices;
    *this->ground_idx = *arvc::inverseIndices(this->cloud_in, this->truss_idx);
    this->ground_idx->insert(this->ground_idx->end(), idx_removed_by_density->begin(), idx_removed_by_density->end());

    this->cloud_out = arvc::extract_indices(this->cloud_in, this->truss_idx, false);
    tmp_cloud = arvc::extract_indices(this->cloud_in, this->ground_idx, false);
    (this->visualize) ?  arvc::visualizeClouds(this->cloud_out,0,0,170, tmp_cloud,0,0,170) : void();


    this->getConfMatrixIndexes();

    PointCloud::Ptr error_cloud (new PointCloud);
    PointCloud::Ptr truss_cloud (new PointCloud);
    PointCloud::Ptr ground_cloud (new PointCloud);
    pcl::IndicesPtr error_idx (new pcl::Indices);

    error_idx->insert(error_idx->end(), this->fp_idx->begin(), this->fp_idx->end());
    error_idx->insert(error_idx->end(), this->fn_idx->begin(), this->fn_idx->end());

    truss_cloud = arvc::extract_indices(cloud_in, this->tp_idx, false);
    ground_cloud = arvc::extract_indices(cloud_in,this->tn_idx, false);
    error_cloud = arvc::extract_indices(cloud_in, error_idx, false);

    pcl::visualization::PCLVisualizer my_vis;
    my_vis.setBackgroundColor(1,1,1);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> truss_color (truss_cloud, 0,255,0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> ground_color (ground_cloud, 100,100,100);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> error_color (error_cloud, 255,0,0);

    my_vis.addPointCloud(truss_cloud, truss_color, "truss_cloud");
    my_vis.addPointCloud(ground_cloud, ground_color, "wrong_cloud");
    my_vis.addPointCloud(error_cloud, error_color, "error_cloud");

    while (!my_vis.wasStopped())
    {
      my_vis.spinOnce(100);
    }
  


    (this->visualize) ?  arvc::visualizeClouds(this->cloud_in,0,0,170, this->cloud_out,0,0,170) : void();


    // Compute metrics
    if(this->compute_metrics)
    {
      this->cm = arvc::computeConfusionMatrix(this->gt_truss_idx, this->gt_ground_idx, this->truss_idx, this->ground_idx);
      this->metricas = arvc::computeMetrics(cm);
    }
    return 0;
  }
};

/*
class segment_planes
{
private:

public:
  fs::path path;
  PointCloud::Ptr cloud_in;
  PointCloud::Ptr cloud_out;
  pcl::IndicesPtr truss_idx;
  pcl::IndicesPtr ground_idx;
  pcl::IndicesPtr gt_truss_idx;
  pcl::IndicesPtr gt_ground_idx;
  pcl::IndicesPtr low_density_idx;

  metrics metricas;
  conf_matrix cm;

  bool visualize;
  bool compute_metrics;

  // Constructor
  segment_planes(fs::path _path){
    this->path = _path;
    this->cloud_in = PointCloud::Ptr (new PointCloud);
    this->cloud_out = PointCloud::Ptr (new PointCloud);
    this->truss_idx = pcl::IndicesPtr (new pcl::Indices);
    this->ground_idx = pcl::IndicesPtr (new pcl::Indices);
    this->gt_truss_idx = pcl::IndicesPtr (new pcl::Indices);
    this->gt_ground_idx = pcl::IndicesPtr (new pcl::Indices);
    this->low_density_idx = pcl::IndicesPtr (new pcl::Indices);

    this->visualize = false;
    this->compute_metrics = false;
  }

  // Destructor
  ~segment_planes(){
    this->path.clear();
    this->cloud_in->clear();
    this->cloud_out->clear();
    this->visualize = false;
    this->compute_metrics = false;
  }

  int
  run()
  {
    PointCloudI::Ptr cloud_in_intensity (new PointCloudI);
    PointCloud::Ptr cloud_in_xyz (new PointCloud);
    PointCloud::Ptr cloud_out_xyz (new PointCloud);
    PointCloud::Ptr tmp_cloud (new PointCloud);
    pcl::IndicesPtr coarse_ground_indices (new pcl::Indices);
    pcl::IndicesPtr coarse_truss_indices (new pcl::Indices);

    pcl::IndicesPtr current_indices (new pcl::Indices);
    pcl::ModelCoefficientsPtr tmp_plane_coefss (new pcl::ModelCoefficients);

    gt_indices ground_truth_indices;

    vector<pcl::PointIndices> regrow_clusters;
    vector<int> valid_clusters;

    cloud_in_intensity = arvc::readCloud(this->path);
    ground_truth_indices = arvc::getGroundTruthIndices(cloud_in_intensity);
    cloud_in_xyz = arvc::parseToXYZ(cloud_in_intensity);
    *this->cloud_in = *cloud_in_xyz;

    regrow_clusters = arvc::regrow_segmentation(this->cloud_in);
    cout << "Regrow clusters size: " << regrow_clusters.size() << endl;

    valid_clusters = arvc::validate_clusters_by_ratio(tmp_cloud, regrow_clusters, 0.3f);
    // valid_clusters = arvc::validate_clusters_by_module(tmp_cloud, regrow_clusters, 0.3f);
    // valid_clusters = arvc::validate_clusters_hybrid(this->cloud_in, regrow_clusters, 0.3f, 0.3f);


    for(int clus_indx : valid_clusters)
      coarse_truss_indices->insert(coarse_truss_indices->end(), regrow_clusters[clus_indx].indices.begin(), regrow_clusters[clus_indx].indices.end());

    // cout << "Coarse truss indices size: " << coarse_truss_indices->size() << endl;
    *this->truss_idx = *coarse_truss_indices;
    *this->ground_idx = *arvc::inverseIndices(this->cloud_in, this->truss_idx);
    this->cloud_out = arvc::extract_indices(this->cloud_in, this->truss_idx, false);
    // cout << "Showing cloud after valid clusters" << endl;
    // (this->visualize) ?  arvc::visualizeCloud(this->cloud_out) : void();



    this->truss_idx = arvc::radius_outlier_removal(this->cloud_in, this->truss_idx, 0.1f, 5, false);
    this->ground_idx = arvc::inverseIndices(this->cloud_in, this->truss_idx);

    this->cloud_out = arvc::extract_indices(this->cloud_in, this->truss_idx, false);

    (this->visualize) ?  arvc::visualizeClouds(this->cloud_in, this->cloud_out) : void();

    // Compute metrics
    if(this->compute_metrics)
    {
      this->cm = arvc::computeConfusionMatrix(this->gt_truss_idx, this->gt_ground_idx, this->truss_idx, this->ground_idx);
      this->metricas = arvc::computeMetrics(cm);
    }

    return 0;
  }
};
*/

int main(int argc, char **argv)
{
  std::cout << YELLOW << "Running your code..." << RESET << std::endl;
  auto start = std::chrono::high_resolution_clock::now();

  vector<float> precision{};
  vector<float> recall{};
  vector<float> f1_score{};
  vector<float> accuracy{};
  vector<int> tp_vector{};
  vector<int> tn_vector{};
  vector<int> fp_vector{};
  vector<int> fn_vector{};

  std::vector<fs::path> path_vector;
  // EVERY CLOUD IN THE CURRENT FOLDER
  if(argc < 2)
  {
    fs::path current_dir = fs::current_path();
    for(const auto &entry : fs::directory_iterator(current_dir))
    {
      if(entry.path().extension() == ".pcd" || entry.path().extension() == ".ply")
        path_vector.push_back(entry.path());
    }

    for(const fs::path &entry : tq::tqdm(path_vector))
    {
      remove_ground rg(entry);
      rg.visualize = false;
      rg.compute_metrics = true;
      rg.run();

      if(rg.compute_metrics)
      {
        accuracy.push_back(rg.metricas.accuracy);
        precision.push_back(rg.metricas.precision);
        recall.push_back(rg.metricas.recall);
        f1_score.push_back(rg.metricas.f1_score);
        tp_vector.push_back(rg.cm.TP);
        tn_vector.push_back(rg.cm.TN);
        fp_vector.push_back(rg.cm.FP);
        fn_vector.push_back(rg.cm.FN);
      }
    }


    cout << endl;
    cout << "Accuracy: " << arvc::mean(accuracy) << endl;
    cout << "Precision: " << arvc::mean(precision) << endl;
    cout << "Recall: " << arvc::mean(recall) << endl;
    cout << "F1 Score: " << arvc::mean(f1_score) << endl;
    cout << "TP: " << arvc::mean(tp_vector) << endl;
    cout << "TN: " << arvc::mean(tn_vector) << endl;
    cout << "FP: " << arvc::mean(fp_vector) << endl;
    cout << "FN: " << arvc::mean(fn_vector) << endl;
    
  }


  // ONLY ONE CLOUD PASSED AS ARGUMENT IN CURRENT FOLDER
  else
  {
    // Get the input data
    fs::path entry = argv[1];
    remove_ground rg(entry);
  
    // segment_planes rg(entry);
    rg.visualize = true;
    rg.compute_metrics = true;
    // rg.run();
    rg.run2();

    if(rg.compute_metrics){
      accuracy.push_back(rg.metricas.accuracy);
      precision.push_back(rg.metricas.precision);
      recall.push_back(rg.metricas.recall);
      f1_score.push_back(rg.metricas.f1_score);
      tp_vector.push_back(rg.cm.TP);
      tn_vector.push_back(rg.cm.TN);
      fp_vector.push_back(rg.cm.FP);
      fn_vector.push_back(rg.cm.FN);
    }


  }

    cout << endl;
    cout << "Accuracy: " << arvc::mean(accuracy) << endl;
    cout << "Precision: " << arvc::mean(precision) << endl;
    cout << "Recall: " << arvc::mean(recall) << endl;
    cout << "F1 Score: " << arvc::mean(f1_score) << endl;
    cout << "TP: " << arvc::mean(tp_vector) << endl;
    cout << "TN: " << arvc::mean(tn_vector) << endl;
    cout << "FP: " << arvc::mean(fp_vector) << endl;
    cout << "FN: " << arvc::mean(fn_vector) << endl;


  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

  std::cout << YELLOW << "Code end!!" << RESET << std::endl;
  std::cout << "Computation Time: " << duration.count() << " ms" << std::endl;
  cout << "Average Computation Time: " << duration.count()/path_vector.size() << " ms" << endl;
  return 0;
}
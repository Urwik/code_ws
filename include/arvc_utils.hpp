#pragma once

// C++
#include <iostream>
#include <algorithm>
#include <filesystem>
#include <chrono>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter_indices.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/radius_outlier_removal.h>

// Visualization
#include <pcl/visualization/pcl_visualizer.h>

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"  
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"

//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

namespace fs = std::filesystem;

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> PointCloudI;

struct metrics
{
  float precision = 0.0;
  float recall = 0.0;
  float f1_score = 0.0;
  float accuracy = 0.0;
};

struct conf_matrix
{
  int TP = 0;
  int FP = 0;
  int TN = 0;
  int FN = 0;
};

struct gt_indices
{
  pcl::Indices ground;
  pcl::Indices truss;
};


namespace arvc
{

  /**
   * @brief Get the Ground Truth object
   * 
   */
  gt_indices
  getGroundTruthIndices(PointCloudI::Ptr &_cloud_intensity)
  {
    gt_indices _gt_indices;
    pcl::PassThrough<pcl::PointXYZI> pt;
    pt.setInputCloud(_cloud_intensity);
    pt.setFilterFieldName("intensity");
    pt.setFilterLimits(0, 0);
    pt.setNegative(false);
    pt.filter(_gt_indices.ground);
    pt.setNegative(true);
    pt.filter(_gt_indices.truss);

    return _gt_indices;
  }


  /**
   * @brief Lee una nube de puntos en formato .pcd o .ply
   * 
   * @param path Ruta de la nube de puntos
   * @return PointCloudI::Ptr 
   */
  PointCloudI::Ptr 
  readCloud (fs::path _path)
  {
    PointCloudI::Ptr _cloud_intensity (new PointCloudI);
    map<string, int> ext_map = {{".pcd", 0}, {".ply", 1}};

    switch (ext_map[_path.extension().string()])
    {
      case 0: {
        pcl::PCDReader pcd_reader;
        pcd_reader.read(_path.string(), *_cloud_intensity);
        break;
      }
      case 1: {
        pcl::PLYReader ply_reader;
        ply_reader.read(_path.string(), *_cloud_intensity);
        break;
      }
      default: {
        std::cout << "Format not compatible, it should be .pcd or .ply" << std::endl;
        break;
      }
    }

    return _cloud_intensity;
  }


  /**
   * @brief Convierte una nube de entrada con intensidad a solo coordenadas XYZ
  */
  PointCloud::Ptr
  parseToXYZ(PointCloudI::Ptr &_cloud_intensity)
  {
    PointCloud::Ptr _cloud_xyz (new PointCloud);
    pcl::copyPointCloud(*_cloud_intensity, *_cloud_xyz);

    return _cloud_xyz;
  }


  /**
   * @brief Realiza agrupaciones de puntos en función de sus normales
   * 
   * @param cloud  Nube de entrada
   * @return std::vector<pcl::PointIndices> Vector con los indices pertenecientes 
   * a cada agrupación 
   */
  vector<pcl::PointIndices>
  regrow_segmentation (PointCloud::Ptr &_cloud_in, pcl::IndicesPtr &_indices)
  {

    // Estimación de normales
    pcl::PointCloud<pcl::Normal>::Ptr _cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(_cloud_in);
    ne.setInputCloud(_cloud_in);
    // ne.setIndices(_indices);
    ne.setSearchMethod(tree);
    ne.setKSearch(30);            // Por vecinos no existen normales NaN
    // ne.setRadiusSearch(0.05);  // Por radio existiran puntos cuya normal sea NaN
    ne.compute(*_cloud_normals);

    // Segmentación basada en crecimiento de regiones
    vector<pcl::PointIndices> _regrow_clusters;
    pcl::RegionGrowing<PointT, pcl::Normal> reg;
    reg.setMinClusterSize (100);
    reg.setMaxClusterSize (25000);
    reg.setSearchMethod (tree);
    reg.setSmoothModeFlag(false);
    reg.setCurvatureTestFlag(true);
    reg.setResidualThreshold(false);
    reg.setCurvatureThreshold(1);
    reg.setNumberOfNeighbours (10);
    reg.setInputCloud (_cloud_in);
    reg.setIndices(_indices);
    reg.setInputNormals (_cloud_normals);
    reg.setSmoothnessThreshold (10.0 / 180.0 * M_PI);
    reg.extract (_regrow_clusters);

    // cout << "Number of clusters: " << _regrow_clusters.size() << endl;

    // Uncomment to visualize cloud
    // pcl::visualization::PCLVisualizer vis ("PCL Visualizer");
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    // color_cloud = reg.getColoredCloud();
    // vis.addPointCloud<pcl::PointXYZRGB>(color_cloud);

    // while (!vis.wasStopped())
    //   vis.spinOnce();

    return _regrow_clusters;
  }


  /**
   * @brief Filtra la nube de puntos en función de los índices pasados como parámetro
   * 
   * @param cloud Nube de entrada
   * @param indices_vec Vector con los índices de los puntos que se quieren extraer
   * @param negative Si es true, se extraen los puntos que no están en indx_vec
   */
  PointCloud::Ptr
  extract_indices(PointCloud::Ptr &cloud, std::vector<pcl::PointIndices> indices_vec, bool negative = false)
  {
    pcl::PointIndices::Ptr indices (new pcl::PointIndices);
    PointCloud::Ptr _cloud_out (new PointCloud);

    for (size_t i = 0; i < indices_vec.size(); i++)
      for(auto index : indices_vec[i].indices)
        indices->indices.push_back(index);
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.setNegative(negative);
    extract.filter(*_cloud_out);

    return _cloud_out;
  }


  /**
   * @brief Filtra la nube de puntos en función de los índices pasados como parámetro
   * 
   * @param cloud Nube de entrada
   * @param indices Indices de los puntos que se quieren extraer
   * @param negative Si es true, se extraen los puntos que no están en indx_vec
   */
  PointCloud::Ptr
  extract_indices (PointCloud::Ptr &_cloud_in, pcl::IndicesPtr &_indices, bool negative = false)
  {
    PointCloud::Ptr _cloud_out (new PointCloud);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(_cloud_in);
    extract.setIndices(_indices);
    extract.setNegative(negative);
    extract.filter(*_cloud_out);

    return _cloud_out;
  }

  /**
   * @brief Filtra la nube de puntos en función de los índices pasados como parámetro
   * 
   * @param cloud 
   * @param optimizeCoefs 
   * @param distThreshold 
   * @param maxIterations 
   * @return pcl::ModelCoefficients::Ptr 
   */
  pcl::ModelCoefficientsPtr 
  compute_planar_ransac (PointCloud::Ptr &_cloud_in, const bool optimizeCoefs,
              float distThreshold = 0.03, int maxIterations = 1000)
  {
    pcl::PointIndices point_indices;
    pcl::SACSegmentation<PointT> ransac;
    pcl::ModelCoefficientsPtr plane_coeffs (new pcl::ModelCoefficients);

    ransac.setInputCloud(_cloud_in);
    ransac.setOptimizeCoefficients(optimizeCoefs);
    ransac.setModelType(pcl::SACMODEL_PLANE);
    ransac.setMethodType(pcl::SAC_RANSAC);
    ransac.setMaxIterations(maxIterations);
    ransac.setDistanceThreshold(distThreshold);
    ransac.segment(point_indices, *plane_coeffs);

    return plane_coeffs;
  }


  /**
   * @brief Filtra la nube de puntos en función de los índices pasados como parámetro
   * 
   * @param cloud 
   * @param coefs 
   * @param distThreshold 
   * @return pcl::PointIndices::Ptr 
   */
  pcl::IndicesPtr
  get_points_near_plane(PointCloud::Ptr &_cloud_in, pcl::ModelCoefficientsPtr &_plane_coeffs, float distThreshold = 0.5f)
  {
    Eigen::Vector4f coefficients(_plane_coeffs->values.data());
    pcl::PointXYZ point;
    pcl::IndicesPtr _plane_inliers (new pcl::Indices);

    for (size_t indx = 0; indx < _cloud_in->points.size(); indx++)
    {
      point = _cloud_in->points[indx];
      float distance = pcl::pointToPlaneDistance(point, coefficients);
      if (pcl::pointToPlaneDistance(point, coefficients) <= distThreshold)
        _plane_inliers->push_back(indx);
    }

    return _plane_inliers;
  }


  /**
   * @brief Computes the eigenvalues of a PointCloud
   * 
   * @param cloud_in 
   * @return Eigen::Vector3f 
   */
  Eigen::Vector3f
  compute_eigenvalues(PointCloud::Ptr &_cloud_in, pcl::IndicesPtr &_indices)
  {
    Eigen::Vector4f xyz_centroid;
    PointCloud::Ptr tmp_cloud (new PointCloud);
    tmp_cloud = arvc::extract_indices(_cloud_in, _indices);
    pcl::compute3DCentroid(*tmp_cloud, xyz_centroid);

    Eigen::Matrix3f covariance_matrix;
    pcl::computeCovarianceMatrixNormalized (*tmp_cloud, xyz_centroid, covariance_matrix); 
    // pcl::computeCovarianceMatrix (*tmp_cloud, xyz_centroid, covariance_matrix); 

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix, Eigen::ComputeEigenvectors);
    Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();

    return eigenValuesPCA;
  }


  /**
   * @brief Filters each cluster by its eigen values
   * 
   * @return Eigen::Vector3f 
   */
  vector<int>
  validate_clusters(PointCloud::Ptr &_cloud_in, vector<pcl::PointIndices> &clusters, float _ratio_threshold)
  {
    cout << GREEN <<"Checking regrow clusters..." << RESET << endl;
    vector<int> valid_clusters;
    valid_clusters.clear();

    int clust_indx = 0;
    for(auto cluster : clusters)
    {
      pcl::IndicesPtr current_cluster (new pcl::Indices);
      *current_cluster = cluster.indices;
      auto eig_values = arvc::compute_eigenvalues(_cloud_in, current_cluster);
      float size_ratio = eig_values(1)/eig_values(2);

      if (size_ratio < _ratio_threshold){
        valid_clusters.push_back(clust_indx);
        cout << "\tValid cluster: " << GREEN << clust_indx << RESET << " with ratio: " << size_ratio << endl;
      }
        
      clust_indx++;
    }

    if (valid_clusters.size() == 0)
      cout << "\tNo valid clusters found" << endl;

    return valid_clusters;
  }


  pcl::IndicesPtr
  ownInverseIndices(PointCloud::Ptr &_cloud_in, pcl::IndicesPtr &_indices_in)
  {
    pcl::IndicesPtr _indices_out (new pcl::Indices);
    _indices_out->resize(_cloud_in->size());
    iota(_indices_out->begin(), _indices_out->end(), 0);

    for (int indx : *_indices_in)
      _indices_out->erase(remove(_indices_out->begin(), _indices_out->end(), indx), _indices_out->end());

    // set_difference(_original_indices->begin(), _original_indices->end(), _indices_in->begin(), _indices_in->end(), inserter(*_indices_out, _indices_out->begin()));

    return _indices_out;
  }


  pcl::IndicesPtr
  inverseIndices(PointCloud::Ptr &_cloud_in, pcl::IndicesPtr &_indices_in)
  {
    pcl::IndicesPtr _indices_out (new pcl::Indices);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(_cloud_in);
    extract.setIndices(_indices_in);
    extract.setNegative(true);
    extract.filter(*_indices_out);

    return _indices_out;
  }

  /**
   * @brief Returns a Voxelized PointCloud
   * 
   * @param _cloud_in 
   * @return pcl::PointCloud<pcl::PointXYZ>::Ptr
   */
  PointCloud::Ptr
  voxel_filter( PointCloud::Ptr &_cloud_in ,float leafSize = 0.1)
  {
    PointCloud::Ptr _cloud_out (new PointCloud);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(_cloud_in);
    sor.setLeafSize(leafSize, leafSize, leafSize);
    sor.filter(*_cloud_out);

    return _cloud_out;
  }

  /**
   * @brief Returns the intersection of two vectors
   * 
   * @param v1 
   * @param v2 
   * @return vector<int> 
   */
  vector<int> 
  intersection(vector<int> v1,
               vector<int> v2){
    vector<int> v3;

    sort(v1.begin(), v1.end());
    sort(v2.begin(), v2.end());

    std::set_intersection(v1.begin(),v1.end(),
                          v2.begin(),v2.end(),
                          back_inserter(v3));
    return v3;
  }

  /**
   * @brief Returns a Voxelized PointCloud
   * 
   * @param cloud_in 
   * @return pcl::PointCloud<pcl::PointXYZ>::Ptr
   */
  metrics
  computeMetrics(conf_matrix cm)
  {
    metrics _metrics;

    _metrics.precision = (float)cm.TP / ((float)cm.TP + (float)cm.FP);
    _metrics.recall = (float)cm.TP / ((float)cm.TP + (float)cm.FN);
    _metrics.f1_score = 2 * (_metrics.precision * _metrics.recall) / (_metrics.precision + _metrics.recall);
    _metrics.accuracy = (float)(cm.TP + cm.TN) / (float)(cm.TP + cm.TN + cm.FP + cm.FN);

    return _metrics;
  }


  /**
   * @brief Computes the confusion matrix from the ground truth and the detected indices
   * 
   * @param gt_truss_idx 
   * @param gt_ground_idx 
   * @param truss_idx 
   * @param ground_idx 
   * @return conf_matrix 
   */
  conf_matrix
  computeConfusionMatrix(pcl::IndicesPtr &gt_truss_idx, pcl::IndicesPtr &gt_ground_idx,  pcl::IndicesPtr &truss_idx, pcl::IndicesPtr &ground_idx)
  {
    conf_matrix _conf_matrix;

    _conf_matrix.TP = arvc::intersection(*gt_truss_idx, *truss_idx).size();
    _conf_matrix.TN = arvc::intersection(*gt_ground_idx, *ground_idx).size();
    _conf_matrix.FP = gt_ground_idx->size() - _conf_matrix.TN;
    _conf_matrix.FN = gt_truss_idx->size() - _conf_matrix.TP;
    
    return _conf_matrix;
  }


  /**
   * @brief Returns a Voxelized PointCloud
   * 
   * @param cloud_in 
   * @return pcl::PointCloud<pcl::PointXYZ>::Ptr
   */
  void 
  writeCloud (pcl::PointCloud<pcl::PointXYZLNormal>::Ptr &cloud_in, fs::path entry)
  {
    pcl::PCDWriter pcd_writer;
    
    fs::path abs_file_path = fs::current_path().parent_path() / "pcd_xyzlabelnormal";
    if (!fs::exists(abs_file_path)) 
      fs::create_directory(abs_file_path);

    std::string filename = entry.stem().string() + ".pcd";

    abs_file_path = abs_file_path / filename;
    pcd_writer.write(abs_file_path.string(), *cloud_in, true);
  }


  /**
   * Visualize current working cloud
  */
  void 
  visualizeCloud ( PointCloud::Ptr &_cloud_in)
  {
    pcl::visualization::PCLVisualizer vis("PCL_Visualizer");

    int v1(0);
    vis.addPointCloud<PointT> (_cloud_in, "Original", v1);

    while(!vis.wasStopped())
      vis.spinOnce(100);

    vis.close();
  }


  void 
  visualizeClouds (PointCloud::Ptr &original_cloud, PointCloud::Ptr &filtered_cloud)
  {
    pcl::visualization::PCLVisualizer vis("PCL_Visualizer");

    int v1(0);
    int v2(0);

    //Define ViewPorts
    vis.createViewPort(0,0,0.5,1, v1);
    vis.createViewPort(0.5,0,1,1, v2);

    vis.removeAllPointClouds();

    vis.addPointCloud<PointT> (original_cloud, "Original", v1);
    vis.addPointCloud<PointT> (filtered_cloud, "Filtered", v2);

    while(!vis.wasStopped())
      vis.spinOnce(100);

    vis.close();
  }


  /**
   * @brief Remove points with less than minNeighbors inside a give radius
   * 
   * @param radius Search sphere radius
   * @param minNeighbors Minimum num of Neighbors to consider a point an inlier
   * @return PointCloud::Ptr Return a PointCloud without low neighbor points
   */
  pcl::IndicesPtr
  radius_outlier_removal (PointCloud::Ptr &_cloud_in, pcl::IndicesPtr &_indices_in, float radius, int minNeighbors, bool negative = false)
  {
    PointCloud::Ptr _cloud_out (new PointCloud);
    pcl::IndicesPtr _indices_out (new pcl::Indices);
    pcl::RadiusOutlierRemoval<PointT> radius_removal;
    radius_removal.setInputCloud(_cloud_in);
    radius_removal.setIndices(_indices_in);
    radius_removal.setRadiusSearch(radius);
    radius_removal.setMinNeighborsInRadius(minNeighbors);
    radius_removal.setNegative(negative);
    radius_removal.filter(*_indices_out);

    return _indices_out;
  }

}
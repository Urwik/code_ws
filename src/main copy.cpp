// C++
#include <iostream>
#include <algorithm>
#include <filesystem>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr
readCloud(std::filesystem::path path)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  std::string file_ext = path.extension();
  if (file_ext == ".pcd") {
    pcl::PCDReader pcd_reader;
    pcd_reader.read(path.string(), *cloud);
  }
  else if (file_ext == ".ply") {
    pcl::PLYReader ply_reader;
    ply_reader.read(path.string(), *cloud);
  }
  else
    std::cout << "Format not compatible, it should be .pcd or .ply" << std::endl;

  return cloud;
}

////////////////////////////////////////////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr 
extractIndices(
  pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
  pcl::PointIndices::Ptr &indices)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud_in);
  extract.setIndices(indices);
  extract.filter(*cloud_out);

  return cloud_out;
}


////////////////////////////////////////////////////////////////////////////////
Eigen::Vector4f 
computeRANSAC(
  pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
  const bool optimizeCoefs,
  float distThreshold = 0.03, 
  int maxIterations = 1000)
{
  pcl::SACSegmentation<pcl::PointXYZ> ransac;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  ransac.setInputCloud(cloud);
  ransac.setOptimizeCoefficients(optimizeCoefs);
  ransac.setModelType(pcl::SACMODEL_PLANE);
  ransac.setMethodType(pcl::SAC_RANSAC);
  ransac.setMaxIterations(maxIterations);
  ransac.setDistanceThreshold(distThreshold);
  ransac.segment(*inliers, *coefficients);


  Eigen::Vector4f plane_coefs;
  for (size_t i = 0; i < 4; i++)
    plane_coefs[i] = coefficients->values[i];
  

  return plane_coefs;
}

/**
 * @brief Calcula la roughness dado un punto.
 * 
 * @param cloud_in Nube de entrada
 * @param index Indice del punto a evaluar.
 * @return float 
 */
float
computeRoughness(
  pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
  int index)
{
  float point_roughness;
  pcl::PointXYZ target_point = cloud_in->points[index];

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud_in);
  int K = 10;

  pcl::PointIndices::Ptr indicesKnearest (new pcl::PointIndices);
  std::vector<float> pointNKNSquaredDistance(K);

  if( kdtree.nearestKSearch (*cloud_in, index, K, indicesKnearest->indices, pointNKNSquaredDistance) > 0 )
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr ngbh_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Vector4f plane_coefs;
    ngbh_cloud = extractIndices(cloud_in, indicesKnearest);
    plane_coefs = computeRANSAC(ngbh_cloud, true, 0.03, 1000);

    point_roughness = pcl::pointToPlaneDistance(target_point, plane_coefs);
  }
  else
    point_roughness = 100;

  return point_roughness;
}





int main(int argc, char const *argv[])
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  cloud_in = readCloud("/media/arvc/data/datasets/ARVCTRUSS/test/ply_xyzlabelnormal/00000.ply");
  float roughness = computeRoughness(cloud_in, 0);
  std::cout << "Roughness: " << roughness << std::endl;
  
  return 0;
}

#pragma once
// C++
#include <iostream>
#include <algorithm>
#include <filesystem>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

typedef pcl::PointXYZL PointL;
typedef pcl::PointXYZLNormal PointLN;
typedef pcl::Normal PointN;

typedef pcl::PointCloud<PointL> PointCloudL;
typedef pcl::PointCloud<PointLN> PointCloudLN;
typedef pcl::PointCloud<PointN> PointCloudN;

// NAMESPACES
namespace fs = std::filesystem;

/**
 * @brief Calcula la normal de cada punto en función de la distribución de sus
 * X vecinos más cercanos
 * 
 * @param cloud_in Nube sobre la que se desean calcular las normales
 * @param neighbours Número de vecinos
 * @return pcl::PointCloud<pcl::Normal>::Ptr Normales asociadas a cada punto
 */

template<typename PointT>
PointCloudLN::Ptr 
normalsByNeighbours(typename pcl::PointCloud<PointT>::Ptr &_cloud_in, int neighbours = 30, bool remove_nan = false)
{
  pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZL>);
  pcl::copyPointCloud(*_cloud_in, *cloud_in);

  PointCloudLN::Ptr cloud_out (new PointCloudLN);
  PointCloudN::Ptr normals (new PointCloudN);
  pcl::search::KdTree<PointL>::Ptr tree (new pcl::search::KdTree<PointL> ());
  pcl::NormalEstimation<PointL, pcl::Normal> ne;
  
  ne.setInputCloud(cloud_in);
  ne.setSearchMethod(tree);
  ne.setKSearch(neighbours);
  ne.compute(*normals);

  pcl::concatenateFields(*cloud_in, *normals, *cloud_out); 

  if (remove_nan) {
    pcl::Indices indices;
    pcl::removeNaNNormalsFromPointCloud<PointT>(*cloud_out, *cloud_out, indices);
  }


  return cloud_out;
}

template<typename PointT>
PointCloudLN::Ptr 
normalsByRadius(typename pcl::PointCloud<PointT>::Ptr &_cloud_in, float _radius = 0.05 , bool remove_nan = true)
{
  pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZL>);
  pcl::copyPointCloud(*_cloud_in, *cloud_in);

  PointCloudLN::Ptr cloud_out (new PointCloudLN);
  PointCloudN::Ptr normals (new PointCloudN);
  pcl::search::KdTree<PointL>::Ptr tree (new pcl::search::KdTree<PointL> ());
  pcl::NormalEstimation<PointL, pcl::Normal> ne;
  
  ne.setInputCloud(cloud_in);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(_radius);
  ne.compute(*normals);

  pcl::concatenateFields(*cloud_in, *normals, *cloud_out); 

  if (remove_nan) {
    pcl::Indices indices;
    pcl::removeNaNNormalsFromPointCloud<PointT>(*cloud_out, *cloud_out, indices);
  }

  return cloud_out;
}




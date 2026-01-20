#pragma once
#include <iostream>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

typedef pcl::PointXYZLNormal PointLN;
typedef pcl::PointCloud<PointLN> PointCloudLN;

PointCloudLN::Ptr randomSampleCloud(PointCloudLN::Ptr &cloud_in, const int target_points)
{
  PointCloudLN::Ptr cloud_out (new PointCloudLN);
  PointCloudLN::Ptr cloud_floor (new PointCloudLN);
  PointCloudLN::Ptr cloud_structure (new PointCloudLN);

  pcl::Indices uselsess;
  pcl::removeNaNFromPointCloud<PointLN>(*cloud_in, *cloud_in, uselsess);
  pcl::removeNaNNormalsFromPointCloud<PointLN>(*cloud_in, *cloud_in, uselsess);

  if((int)cloud_in->points.size() < target_points)
  {
    std::cout << "Removing cloud with: " << cloud_in->points.size() << std::endl;
  }
  else
  {
    pcl::RandomSample<PointLN> rs;
    rs.setInputCloud(cloud_in);
    rs.setSample(target_points);
    rs.setSeed(std::rand());
    rs.filter(*cloud_out);
  }

  return cloud_out;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr random_sample_cloud<PointT>(typename pcl::PointCloud<PointT>::Ptr &cloud_in, const int target_points)
{
  typename pcl::PointCloud<PointT>::Ptr cloud_out (new pcl::PointCloud<PointT>);

  pcl::Indices uselsess;
  std::cout << "Removing NaNs from cloud..." << std::endl;
  pcl::removeNaNFromPointCloud<PointT>(*cloud_in, *cloud_out, uselsess);

  if(cloud_out->points.size() > target_points)
  {
    std::cout << "Sampling cloud to " << target_points << " points..." << std::endl;

    typename  pcl::RandomSample<PointT> rs;
    rs.setInputCloud(cloud_out);
    rs.setSample(target_points);
    rs.filter(*cloud_out);
  }

  std::cout << "Initial cloud size: " << cloud_in->points.size() << std::endl;
  std::cout << "Number of NaNs removed: " << uselsess.size() << std::endl;
  std::cout << "Final cloud size: " << cloud_out->points.size() << std::endl;

  return cloud_out;
}
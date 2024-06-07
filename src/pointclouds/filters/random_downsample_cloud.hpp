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

  if((int)cloud_in->points.size() < target_points)
  {
    std::cout << "\nThe number of target points is higher than the actual points. Reduce the number of target points" << std::endl;
    // std::cout << "Exiting..."<< std::endl;
    // exit(EXIT_FAILURE);
  }


  else
  {
    pcl::Indices uselsess;
    pcl::removeNaNFromPointCloud<PointLN>(*cloud_in, *cloud_in, uselsess);

    pcl::RandomSample<PointLN> rs;
    rs.setInputCloud(cloud_in);
    rs.setSample(target_points);
    rs.setSeed(std::rand());
    rs.filter(*cloud_out);
  }

  return cloud_out;
}

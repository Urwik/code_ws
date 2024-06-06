#pragma once
#include <iostream>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

typedef pcl::PointXYZLNormal PointLN;
typedef pcl::PointCloud<PointLN> PointCloud;

PointCloud::Ptr randomSampleCloud(PointCloud::Ptr &cloud_in, const int target_points)
{
  PointCloud::Ptr cloud_out (new PointCloud);
  PointCloud::Ptr cloud_floor (new PointCloud);
  PointCloud::Ptr cloud_structure (new PointCloud);

  int n_points_to_remove = cloud_in->points.size() - target_points;

  if(n_points_to_remove < 0)
  {
    std::cout << "\nThe number of target points is higher than the actual points. Reduce the number of target points" << std::endl;
    // std::cout << "Exiting..."<< std::endl;
    // exit(EXIT_FAILURE);
  }
  else
  {
    pcl::Indices uselsess;
    pcl::removeNaNFromPointCloud<PointLN>(*cloud_in, *cloud_in, uselsess);

    pcl::PassThrough<PointLN> pass;
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("label");
    pass.setFilterLimits(0, 0);
    pass.setNegative(false);
    pass.filter(*cloud_floor);
    pass.setNegative(true);
    pass.filter(*cloud_structure);

    pcl::RandomSample<PointLN> rs;
    rs.setInputCloud(cloud_floor);
    rs.setSample((unsigned int) cloud_floor->points.size() - n_points_to_remove);
    rs.setSeed(std::rand());
    rs.filter(*cloud_floor);

    *cloud_out = *cloud_structure + *cloud_floor;
  }

  return cloud_out;
}

#include <iostream>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>


namespace fs = std::filesystem;

//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

typedef pcl::PointXYZL PointL;
typedef pcl::PointCloud<PointL> PointCloud;

PointCloud::Ptr downSampleCloud(PointCloud::Ptr &cloud_in, const int target_points, const std::string filename)
{
  PointCloud::Ptr cloud_out (new PointCloud);
  PointCloud::Ptr cloud_floor (new PointCloud);
  PointCloud::Ptr cloud_structure (new PointCloud);

  int points_remove = cloud_in->points.size() - target_points;

  if(points_remove < 0)
  {
    std::cout << "\nThe number of target points is higher than the actual points "
    "in the cloud "<< filename <<", reduce the number of target points" << std::endl;
    // exit(EXIT_FAILURE);
  }
  else
  {
    pcl::Indices uselsess;
    pcl::removeNaNFromPointCloud<PointT>(*cloud_in, *cloud_in, uselsess);
    pcl::removeNaNNormalsFromPointCloud<PointT>(*cloud_in, *cloud_in, uselsess);

    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("label");
    pass.setFilterLimits(0, 0);
    pass.setNegative(false);
    pass.filter(*cloud_floor);
    pass.setNegative(true);
    pass.filter(*cloud_structure);

    pcl::RandomSample<PointT> rs;
    rs.setInputCloud(cloud_floor);
    rs.setSample((unsigned int) cloud_floor->points.size()- points_remove);
    rs.setSeed(std::rand());
    rs.filter(*cloud_floor);

    *cloud_out = *cloud_structure + *cloud_floor;
  }

  return cloud_out;
}
#pragma once
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// ************************************************************************** //

////////////////////////////////////////////////////////////////////////////////

// pcl::PointCloud<pcl::Normal>::Ptr getNormals(PointCloud::Ptr &cloud_in)
// {
//   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//   cloud_normals->points.resize(cloud_in->points.size());

//   for (size_t i = 0; i < cloud_in->points.size(); i++)
//   {
//     cloud_normals->points[i].normal_x = cloud_in->points[i].normal_x;
//     cloud_normals->points[i].normal_y = cloud_in->points[i].normal_y;
//     cloud_normals->points[i].normal_z = cloud_in->points[i].normal_z;
//     cloud_normals->points[i].curvature = cloud_in->points[i].curvature;
//   }

//   return cloud_normals;  
// }


template<typename PointT>
void plotCloudWithNormals(typename pcl::PointCloud<PointT>::Ptr &cloud_in)
{
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::copyPointCloud(*cloud_in, *cloud_normals);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*cloud_in, *cloud_xyz);

  // cloud_normals = getNormals(cloud_in);

  pcl::visualization::PCLVisualizer vis;
  vis.setBackgroundColor(0.1, 0.1, 0.1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color (cloud_xyz, 0, 155, 0);

  vis.addPointCloud<pcl::PointXYZ>(cloud_xyz, cloud_color, "cloud");
  vis.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_xyz, cloud_normals, 1, 0.1, "normals");

  while (!vis.wasStopped())
    vis.spinOnce(100);
  
}


#include <iostream>
#include <filesystem>
#include <thread>

#include "arvc_utils.hpp"

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

using namespace std;
namespace fs = std::filesystem;

int main(int argc, char **argv)
{
  fs::path current_path = fs::current_path();
  PointCloudL::Ptr infered_cloud (new PointCloudL);
  PointCloudL::Ptr gt_cloud (new PointCloudL);
  PointCloud::Ptr cloud_in (new PointCloud);
  gt_indices _gt_idx;
  gt_indices _i_idx;
  cm_indices _cm_indices;
  _cm_indices.tp_idx.reset(new pcl::Indices);
  _cm_indices.tn_idx.reset(new pcl::Indices);
  _cm_indices.fp_idx.reset(new pcl::Indices);
  _cm_indices.fn_idx.reset(new pcl::Indices);

  fs::path gt_path("/home/arvc/Desktop/INFERED_CLOUDS/gt/00567.ply");
  fs::path infered_path("/home/arvc/Desktop/INFERED_CLOUDS/infered/00567.ply");

  gt_cloud = arvc::readCloudWithLabel(gt_path);
  _gt_idx = arvc::getGroundTruthIndices(gt_cloud);
  
  infered_cloud = arvc::readCloudWithLabel(infered_path);
  _i_idx = arvc::getGroundTruthIndices(infered_cloud);

  cloud_in = arvc::parseToXYZ(gt_cloud);

  //   // Definir el factor de escala
  // float scalingFactor = 10.0;  // Escala de 2 (duplicar el tamaño)

  // // Aplicar la transformación de escala a la nube de puntos
  // Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  // transform.scale(scalingFactor);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr scaledCloud(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::transformPointCloud(*cloud_in, *cloud_in, transform);

  _cm_indices = arvc::compute_cm_indices(_gt_idx.truss, _gt_idx.ground, _i_idx.truss, _i_idx.ground);

  PointCloud::Ptr error_cloud (new PointCloud);
  PointCloud::Ptr truss_cloud (new PointCloud);
  PointCloud::Ptr ground_cloud (new PointCloud);
  pcl::IndicesPtr error_idx (new pcl::Indices);

  error_idx->insert(error_idx->end(), _cm_indices.fp_idx->begin(), _cm_indices.fp_idx->end());
  error_idx->insert(error_idx->end(), _cm_indices.fn_idx->begin(), _cm_indices.fn_idx->end());

  truss_cloud = arvc::extract_indices(cloud_in, _cm_indices.tp_idx, false);
  ground_cloud = arvc::extract_indices(cloud_in, _cm_indices.tn_idx, false);
  error_cloud = arvc::extract_indices(cloud_in, error_idx, false);

  pcl::visualization::PCLVisualizer my_vis;
  my_vis.setBackgroundColor(1,1,1);
  
  my_vis.addCoordinateSystem(0.8, "sensor_origin");
  auto pos = cloud_in->sensor_origin_;
  auto ori = cloud_in->sensor_orientation_;
  
  Eigen::Vector3f position(pos[0], pos[1], pos[2]);
  my_vis.addCube(position, ori, 0.3, 0.3, 0.3, "sensor_origin");


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

  return 0;  
}



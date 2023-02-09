#include <filesystem>
#include "pointclouds/arvc_utils.cpp"



int main(int argc, char const *argv[])
{
  pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZLNormal>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  cloud_in = arvc::readCloud("/media/arvc/data/datasets/ARVCTRUSS/train/ply_xyzlabelnormal/00000.ply");
  cloud_xyz = arvc::parseXYZLNormalToXYZ(cloud_in);
  float roughness = arvc::computeRoughness(cloud_xyz, 1);
  std::cout << "Roughness: " << roughness << std::endl;
  
  return 0;
}

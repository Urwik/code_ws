#include <filesystem>
#include "pointclouds/arvc_utils.cpp"



int main(int argc, char const *argv[])
{
  pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZLNormal>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  cloud_in = arvc::readCloud("/media/arvc/data/datasets/ARVCTRUSS/code_test/ply_xyzlabelnormal/00000.ply");

  std::vector<int> labels;
  labels.resize(cloud_in->points.size());
  for (size_t i = 0; i < cloud_in->size(); i++)
  {
    if(cloud_in->points[i].label > 0)
      labels[i]= 1;
    else
      labels[i]=0;
  }
  
  int positives = std::count(labels.begin(), labels.end(), 1);

  float weight = positives/(cloud_in->points.size());
  std::cout << "Num of Positives: " << positives << std::endl;
  std::cout << "Pos Weight: " << weight << std::endl;

  return 0;
}


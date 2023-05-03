#include <filesystem>

#include "arvc_utils.cpp"
#include "tqdm.hpp"

namespace fs = std::filesystem;

int main(int argc, char const *argv[])
{
  pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZLNormal>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<float> roughness;

  auto start = std::chrono::high_resolution_clock::now();

  fs::path current_dir = fs::current_path();

  // EVERY CLOUD IN THE CURRENT FOLDER
  if(argc < 2)
  {
    std::vector<fs::path> path_vector;
    for(const auto &entry : fs::directory_iterator(current_dir))
    {
      if(entry.path().extension() == ".pcd" || entry.path().extension() == ".ply")
        path_vector.push_back(entry.path());
    }

    for(const fs::path &entry : tq::tqdm(path_vector))
    {
      cloud_in = arvc::readCloud(entry);
      cloud_xyz = arvc::parseXYZLNormalToXYZ(cloud_in);
      for (size_t i = 0; i < cloud_xyz->size(); i++)
        roughness.push_back(arvc::computeRoughness(cloud_xyz, i));

    }
  }
  // ONLY ONE CLOUD PASSED AS ARGUMENT IN CURRENT FOLDER
  else
  {
    fs::path entry = argv[1];
    cloud_in = arvc::readCloud(entry);
    cloud_xyz = arvc::parseXYZLNormalToXYZ(cloud_in);
    for (size_t i = 0; i < cloud_xyz->size(); i++)
      roughness.push_back(arvc::computeRoughness(cloud_xyz, i));

    cloud_rgb = arvc::getColoredCloudFromRoughness(cloud_xyz, roughness);

    pcl::visualization::PCLVisualizer vis;
    vis.addPointCloud<pcl::PointXYZRGB>(cloud_rgb);

    while(!vis.wasStopped())
      vis.spinOnce(100);

  }

  std::cout << GREEN << "COMPLETED!!" << RESET << std::endl;
  return 0;
}
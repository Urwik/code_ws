// C++
#include <iostream>
#include <algorithm>
#include <filesystem>
#include <chrono>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include "arvc_utils.cpp"

#include <pcl/filters/passthrough.h>

// Visualization
#include <pcl/visualization/cloud_viewer.h>

#include "tqdm.hpp"

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"  
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
// ************************************************************************** //
namespace fs = std::filesystem;
using namespace std;


////////////////////////////////////////////////////////////////////////////////
PointCloud::Ptr 
readCloud(fs::path path)
{
  PointCloud::Ptr cloud (new PointCloud);

  std::string file_ext = path.extension();

  if (file_ext == ".pcd")
  {
    pcl::PCDReader pcd_reader;
    pcd_reader.read(path.string(), *cloud);
  }
  else if (file_ext == ".ply")
  {
    pcl::PLYReader ply_reader;
    ply_reader.read(path.string(), *cloud);
  }
  else
    std::cout << "Format not compatible, it should be .pcd or .ply" << std::endl;


  return cloud;
}


////////////////////////////////////////////////////////////////////////////////
PointCloud::Ptr 
passThrough(PointCloud::Ptr &cloud, Eigen::Vector3f &leaf)
{
  vector<string> fields{"x","y","z"};
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(cloud);
  

}




////////////////////////////////////////////////////////////////////////////////
PointCloud::Ptr 
extractIndices(PointCloud::Ptr &cloud, pcl::PointIndices::Ptr &indices, bool setNegative)
{
  PointCloud::Ptr cloud_out (new PointCloud);
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(indices);
  extract.setNegative(setNegative);
  extract.filter(*cloud_out);

  return cloud_out;
}


////////////////////////////////////////////////////////////////////////////////
void 
writeCloud(pcl::PointCloud<pcl::PointXYZLNormal>::Ptr &cloud_in, fs::path entry)
{
  pcl::PLYWriter ply_writer;
  
  fs::path abs_file_path = fs::current_path().parent_path() / "ply_xyzlabelnormal_gf_RANSAC";
  if (!fs::exists(abs_file_path)) 
    fs::create_directory(abs_file_path);

  std::string filename = entry.stem().string() + ".ply";

  abs_file_path = abs_file_path / filename;
  ply_writer.write(abs_file_path.string(), *cloud_in, true);
}


////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS); //OCULTA TODOS LOS MENSAJES DE PCL

  auto start = std::chrono::high_resolution_clock::now();
  PointCloud::Ptr cloud_in (new PointCloud);
  PointCloud::Ptr cloud_filtered (new PointCloud);
  PointCloud::Ptr cloud_out (new PointCloud);

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
      cloud_in = readCloud(entry);
      cloud_filtered = arvc::voxelFilter(cloud_in);
      pcl::ModelCoefficients::Ptr coeffs = applyRANSAC2(cloud_filtered, true, 1, 1000);
      pcl::PointIndices::Ptr indices = getPointsNearPlane(cloud_in, coeffs, 1);
      cloud_out = extractIndices(cloud_in, indices, true);
      writeCloud(cloud_out, entry);
    }
    std::cout << std::endl; // Salto de linea despues de tqdm 

    // COMPUTATION TIME
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Computation Time: " << duration.count() << " ms" << std::endl;
  }
  // ONLY ONE CLOUD PASSED AS ARGUMENT IN CURRENT FOLDER
  else
  {
    fs::path entry = argv[1];
    cloud_in = readCloud(entry);

    pcl::PointIndices::Ptr indices = applyRANSAC(cloud_in, true, 0.5, 1000);
    cloud_out = extractIndices(cloud_in, indices, true);

    // COMPUTATION TIME
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Computation Time: " << duration.count() << " ms" << std::endl;


    // VISUALIZATION
    pcl::visualization::PCLVisualizer vis ("PCL Visualizer");
    
    // Define ViewPorts
    int v1(0);
    int v2(0);
    int v3(0);
    int v4(0);

    // 2 HORIZONTAL VIEWPORTS
    // vis.createViewPort(0,0,0.5,1, v1);
    // vis.createViewPort(0.5,0,1,1, v2);

    // 3 HORIZONTAL VIEWPORTS
    // vis.createViewPort(0,0,0.33,1, v1);
    // vis.createViewPort(0.33,0,0.66,1, v2);
    // vis.createViewPort(0.66,0,1,1, v3);

    // 4 DISTRIBUTED VIEWPORTS
    vis.createViewPort(0,0.5,0.5,1, v1);
    vis.createViewPort(0.5,0.5,1,1, v2);
    vis.createViewPort(0,0,0.5,0.5, v3);
    vis.createViewPort(0.5,0,1,0.5, v4);

    vis.addPointCloud<PointT>(cloud_in, "original_cloud", v1);
    vis.addPointCloud<PointT>(cloud_out, "ransac_cloud", v2);


    // vis.addPointCloud<pcl::PointXYZRGB>(regrow_data.colored_cloud, "regrow_cloud", v2);
    // vis.addPointCloud<pcl::PointXYZRGB>(regrow_data2.colored_cloud, "regrow_cloud2", v3);

    // pcl::visualization::PointCloudColorHandlerCustom<PointT> no_ground_color(no_ground_cloud, 0, 255, 0);
    // vis.addPointCloud<PointT> (no_ground_cloud, no_ground_color, "no ground", v4);

    while(!vis.wasStopped())
      vis.spinOnce(100);
  }

  std::cout << GREEN << "COMPLETED!!" << RESET << std::endl;
  return 0;
}
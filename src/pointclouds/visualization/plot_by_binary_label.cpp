#include <iostream>
#include <filesystem>
#include <thread>

// #include "arvc_utils.hpp"

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

// Type Definitions ////////////////////////////////////////////////////////////
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointXYZL PointI;
typedef pcl::PointCloud<PointI> PointCloudI;
typedef pcl::PointXYZL PointL;
typedef pcl::PointCloud<PointL> PointCloudL;

using namespace std;
namespace fs = std::filesystem;

  /**
   * @brief Lee una nube de puntos en formato .pcd o .ply
   * 
   * @param path Ruta de la nube de puntos
   * @return PointCloudI::Ptr 
   */
  PointCloudL::Ptr 
  readCloud (fs::path _path)
  {
    PointCloudL::Ptr _cloud_label (new PointCloudL);
    map<string, int> ext_map = {{".pcd", 0}, {".ply", 1}};

    switch (ext_map[_path.extension().string()])
    {
      case 0: {
        pcl::PCDReader pcd_reader;
        pcd_reader.read(_path.string(), *_cloud_label);
        break;
      }
      case 1: {
        pcl::PLYReader ply_reader;
        ply_reader.read(_path.string(), *_cloud_label);
        break;
      }
      default: {
        std::cout << "Format not compatible, it should be .pcd or .ply" << std::endl;
        break;
      }
    }

    return _cloud_label;
  }



void plotCloud(PointCloudL::Ptr &cloud)
{

  PointCloudL::Ptr truss_cloud (new PointCloudL);
  PointCloudL::Ptr env_cloud (new PointCloudL);

  // Create the filtering object
  pcl::PassThrough<PointL> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("label");
  pass.setFilterLimits (0.0, 0.0);
  pass.setNegative(false);
  pass.filter (*env_cloud);
  pass.setNegative(true);
  pass.filter(*truss_cloud);

  // writeCloud(truss_cloud);

  // Definir el factor de escala
  float scalingFactor = 10.0;  // Escala de 2 (duplicar el tamaño)

  // Aplicar la transformación de escala a la nube de puntos
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.scale(scalingFactor);
  pcl::PointCloud<pcl::PointXYZ>::Ptr scaledCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*env_cloud, *env_cloud, transform);
  pcl::transformPointCloud(*truss_cloud, *truss_cloud, transform);



  pcl::visualization::PCLVisualizer visualizer;
  visualizer.setBackgroundColor(0,0,0);
  visualizer.addCoordinateSystem(0.3, "sensor_origin");
  auto pos = cloud->sensor_origin_;
  auto ori = cloud->sensor_orientation_;
  

  Eigen::Vector3f position(pos[0], pos[1], pos[2]);
  visualizer.addCube(position, ori, 0.1, 0.1, 0.1, "sensor_origin");
  pcl::visualization::PointCloudColorHandlerCustom<PointL> truss_color (truss_cloud, 0,200,0);
  visualizer.addPointCloud<PointL>(truss_cloud, truss_color, "truss_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointL> env_color (env_cloud, 150, 150, 150);
  visualizer.addPointCloud<PointL>(env_cloud, env_color, "env_cloud");

  visualizer.addCoordinateSystem(0.01,0,0,0, "sensor_origin");


  while (!visualizer.wasStopped())
    visualizer.spinOnce(100);

}

/* OTHER WAY TO PLOT CLOUD IN COLORS
myRGB randRGB()
{
  myRGB color;
  color.r = rand() % 256;
  color.g = rand() % 256;
  color.b = rand() % 256;

  return color;
}


void plotCloud(fs::path path_to_cloud, int min_points = 20, int max_points = 1000)
{
  PointCloud::Ptr cloud (new PointCloud);
  PointCloud::Ptr tmp_cloud (new PointCloud);
  PointCloud::Ptr truss_cloud (new PointCloud);


  pcl::PointIndices::Ptr indices (new pcl::PointIndices ());
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);

  std::string ext = path_to_cloud.extension();
  if(ext == ".pcd")
  {
    pcl::PCDReader reader;
    reader.read(path_to_cloud, *cloud);
  }  
  else
  {
    pcl::PLYReader reader;
    reader.read(path_to_cloud, *cloud);
  }
    
  std::cout << path_to_cloud.stem().string() << std::endl;
    
  indices.reset(new pcl::PointIndices);
  indices = findValue(cloud, 0);
  // std::cout << "Index " << i << ": " << indices->indices.size() << std::endl;

  extract.setIndices(indices);
  extract.filter(*tmp_cloud);

  extract.setNegative(true);
  extract.filter(*truss_cloud);

  pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color (tmp_cloud, 100, 100, 100);
  visualizer->addPointCloud<PointT>(tmp_cloud, single_color, "tmp_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> truss_color (truss_cloud, 0, 255, 0);
  visualizer->addPointCloud<PointT>(truss_cloud, truss_color, "truss_cloud");
}

void getlastkey()
{
  while(!visualizer->wasStopped())
  {
    std::getchar();
    button = true;
  }
}
*/

int main(int argc, char **argv)
{
  fs::path current_path = fs::current_path();
  PointCloudL::Ptr cloud_in (new PointCloudL);

  if(argc < 2)
  {
    for(const auto &entry : fs::directory_iterator(current_path))
    {
    }
  }


  else
  {
    fs::path entry = argv[1];
    cloud_in = readCloud(entry);
    plotCloud(cloud_in);
  }

  return 0;  
}



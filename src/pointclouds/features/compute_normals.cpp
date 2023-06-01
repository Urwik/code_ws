// C++
#include <iostream>
#include <algorithm>
#include <filesystem>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>

#include "tqdm.hpp"
#include "arvc_utils.hpp"

// DEFINES 
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"  
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"

// TYPE DEFINITIONS
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// NAMESPACES
namespace fs = std::filesystem;


/**
 * @brief Calcula la normal de cada punto en función de la distribución de sus
 * X vecinos más cercanos
 * 
 * @param cloud_in Nube sobre la que se desean calcular las normales
 * @param neighbours Número de vecinos
 * @return pcl::PointCloud<pcl::Normal>::Ptr Normales asociadas a cada punto
 */
pcl::PointCloud<pcl::PointNormal>::Ptr 
computeNormalsByNeighbours(PointCloud::Ptr &_cloud_in, int neighbours = 30)
{
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  
  ne.setInputCloud(_cloud_in);
  ne.setSearchMethod(tree);
  ne.setKSearch(neighbours);
  ne.compute(*normals);

  pcl::concatenateFields(*_cloud_in, *normals, *cloud_out); 

  return cloud_out;
}


pcl::PointCloud<pcl::PointNormal>::Ptr 
computeNormalsByRadius(PointCloud::Ptr &_cloud_in, float _radius = 0.05)
{
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  
  ne.setInputCloud(_cloud_in);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(_radius);
  ne.compute(*normals);

  pcl::concatenateFields(*_cloud_in, *normals, *cloud_out); 

  return cloud_out;
}

/**
 * @brief Escribe la nube de puntos en formato .ply en un fichero
 * 
 * @param cloud_in Nube de entrada
 * @param entry Archivo de entrada
 */
void writeCloud(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_in, fs::path entry, std::string _name = "cloud", std::string _extension = ".ply")
{
  
  fs::path abs_file_path = fs::current_path().parent_path() / "test_normals";
  if (!fs::exists(abs_file_path)) 
    fs::create_directory(abs_file_path);

  std::string filename = entry.stem().string() + _name + _extension;
  abs_file_path = abs_file_path / filename;

  if(_extension == ".ply"){
    pcl::PLYWriter ply_writer;
    ply_writer.write(abs_file_path.string(), *cloud_in, true, false);
  }
  else if(_extension == ".pcd"){
    pcl::PCDWriter pcd_writer;
    pcd_writer.write(abs_file_path.string(), *cloud_in, true);
  }
  else
    std::cout << RED << "ERROR: " << RESET << "Extension not supported" << std::endl;
}


  void 
  visualizeClouds (pcl::PointCloud<pcl::PointNormal>::Ptr &_neighbour_cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &_radius_cloud)
  {
    pcl::visualization::PCLVisualizer vis("PCL_Visualizer");

    int v1(0);
    int v2(0);

    //Define ViewPorts
    vis.createViewPort(0,0,0.5,1, v1);
    vis.createViewPort(0.5,0,1,1, v2);

    vis.removeAllPointClouds();


    vis.addPointCloud<pcl::PointNormal> (_neighbour_cloud, "Neighbour", v1);
    vis.addPointCloud<pcl::PointNormal> (_radius_cloud, "Radius", v2);

    while(!vis.wasStopped())
      vis.spinOnce(100);

    vis.close();
  }

int main(int argc, char **argv)
{
  auto start = std::chrono::high_resolution_clock::now();
  PointCloud::Ptr cloud_in (new PointCloud);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_neighbour (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_radius (new pcl::PointCloud<pcl::PointNormal>);


  // Multiple files
  if(argc < 2)
  {
    fs::path current_dir = fs::current_path();
    std::vector<fs::path> path_vector;
    for(const auto &entry : fs::directory_iterator(current_dir))
    {
      if(entry.path().extension() == ".pcd" || entry.path().extension() == ".ply")
        path_vector.push_back(entry.path());
    }

    for(const fs::path &entry : tq::tqdm(path_vector))
    {
      cloud_in = arvc::readCloud(entry);

    }
  }

  // Single file
  else
  {
    fs::path entry = argv[1];
    cloud_in = arvc::readCloud(entry);
    // cloud_in_label = parseToXYZLabel(cloud_in);
    cloud_normals_neighbour = computeNormalsByNeighbours(cloud_in, 30);
    cloud_normals_radius = computeNormalsByRadius(cloud_in, 0.1);

    // visualizeClouds(cloud_normals_neighbour, cloud_normals_radius);
    writeCloud(cloud_normals_neighbour, entry, "_neighbour", ".pcd");
    writeCloud(cloud_normals_radius, entry, "_radius", ".pcd");
  }

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "Computation Time: " << duration.count() << " ms" << std::endl;
  
  std::cout << GREEN << "COMPLETED!!" << RESET << std::endl;
  return 0;
}




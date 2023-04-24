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

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/radius_outlier_removal.h>


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
namespace fs = std::filesystem;
using namespace std;

////////////////////////////////////////////////////////////////////////////////
struct filterGroundClouds
{
  pcl::PointCloud<pcl::PointXYZLNormal> ground;
  pcl::PointCloud<pcl::PointXYZLNormal> no_ground;
};


////////////////////////////////////////////////////////////////////////////////
struct regrow
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
  std::vector <pcl::PointIndices> clusters; 
};

struct rgb{
  int r;
  int g;
  int b;
};

////////////////////////////////////////////////////////////////////////////////
PointCloud::Ptr 
readCloud (fs::path path)
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


/*******************************************************************************
 * @brief Realiza agrupaciones de puntos en función de sus normales
 * 
 * @param cloud  Nube de entrada
 * @return std::vector<pcl::PointIndices> Vector con los indices pertenecientes 
 * a cada agrupación 
 */
std::vector<pcl::PointIndices> regrow_clustering (PointCloud::Ptr &cloud_in)
{
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

  // Estimación de normales
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_in);
  ne.setInputCloud(cloud_in);
  ne.setSearchMethod(tree);
  ne.setKSearch(40); // Por vecinos no existen normales NaN
  // ne.setRadiusSearch(0.05); // Por radio existiran puntos cuya normal sea NaN
  ne.compute(*normals);

  // Segmentación basada en crecimiento de regiones
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  std::vector <pcl::PointIndices> clusters;
  reg.setMinClusterSize (100);
  reg.setMaxClusterSize (25000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (10);
  reg.setInputCloud (cloud_in);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold (5.0 * (M_PI / 180));
  reg.setCurvatureThreshold (1);
  reg.extract (clusters);

  return clusters;
}


////////////////////////////////////////////////////////////////////////////////
PointCloud::Ptr 
extractIndices (PointCloud::Ptr &cloud, pcl::PointIndices::Ptr &indices)
{
  PointCloud::Ptr cloud_out (new PointCloud);
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(indices);
  extract.filter(*cloud_out);

  return cloud_out;
}


////////////////////////////////////////////////////////////////////////////////
// Plot clouds in two viewports
void 
visualizeClouds (PointCloud::Ptr &original_cloud, PointCloud::Ptr &filtered_cloud)
{
  pcl::visualization::PCLVisualizer vis("PCL_Visualizer");

  int v1(0);
  int v2(0);

  //Define ViewPorts
  vis.createViewPort(0,0,0.5,1, v1);
  vis.createViewPort(0.5,0,1,1, v2);

  vis.removeAllPointClouds();

  vis.addPointCloud<PointT> (original_cloud, "Original", v1);
  vis.addPointCloud<PointT> (filtered_cloud, "Filtered", v2);

  while(!vis.wasStopped())
    vis.spinOnce(100);

}


////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  cout << GREEN << "Computing clusters with region growing" << RESET << endl;
  auto start = std::chrono::high_resolution_clock::now();

  PointCloud::Ptr cloud_in (new PointCloud);
  vector<pcl::PointIndices> clusters;

  // EVERY CLOUD IN THE CURRENT FOLDER
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
    }

  }
  // ONLY ONE CLOUD PASSED AS ARGUMENT IN CURRENT FOLDER
  else
  {
    fs::path entry = argv[1];
    cloud_in = readCloud(entry);
    clusters = regrow_clustering(cloud_in);
    
    pcl::PointIndices::Ptr tmp_indices (new pcl::PointIndices);
    PointCloud::Ptr tmp_cloud (new PointCloud);
    pcl::visualization::PCLVisualizer::Ptr vis (new pcl::visualization::PCLVisualizer ("Cloud Visualizer"));
    pcl::visualization::PointCloudColorHandlerCustom<PointT> orig_color(cloud_in, 75, 75, 75);
    vis->addPointCloud<PointT>(cloud_in, orig_color, "original");
    stringstream ss;
    int clust_num = 0;
    rgb color; 

    for (pcl::PointIndices indices : clusters)
    {
      *tmp_indices = indices;
      tmp_cloud = extractIndices(cloud_in, tmp_indices);

      // Generate name for each cluster
      ss.str("");
      ss << "cluster" << clust_num;
      // Generate color for each cluster
      color.r = rand() % 255 + 100;
      color.g = rand() % 255 + 100;
      color.b = rand() % 255 + 100;


      pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color(tmp_cloud, color.r, color.g, color.b);
      vis->addPointCloud<PointT> (tmp_cloud, cloud_color, ss.str());
    
      clust_num++;

    }
    
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    std::cout << "Computation Time: " << duration.count() << " ms" << std::endl;
    std::cout << GREEN << "COMPLETED!!" << RESET << std::endl;

    while(!vis->wasStopped())
      vis->spinOnce(100);

  //////////////////////////////////////////////////////////////////////////////
  }

  return 0;
}


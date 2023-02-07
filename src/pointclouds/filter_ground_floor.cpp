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
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>


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
typedef pcl::PointXYZLNormal PointT;
typedef pcl::PointCloud<PointT> PointCloud;
// ************************************************************************** //
namespace fs = std::filesystem;

////////////////////////////////////////////////////////////////////////////////

struct return_clouds
{
  pcl::PointCloud<pcl::PointXYZLNormal> ground;
  pcl::PointCloud<pcl::PointXYZLNormal> no_ground;
};


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


std::vector<pcl::PointIndices> 
computeClusters(PointCloud::Ptr &cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  pcl::copyPointCloud(*cloud, *cloud_xyz);

  //***** Estimación de normales *********************************************//
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_xyz);
  ne.setInputCloud(cloud_xyz);
  ne.setSearchMethod(tree);
  ne.setKSearch(20);
  // ne.setRadiusSearch(0.015);
  ne.compute(*cloud_normals);

  //***** Segmentación basada en crecimiento de regiones *********************//
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  std::vector <pcl::PointIndices> clusters;
  reg.setMinClusterSize (100);
  reg.setMaxClusterSize (25000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (10);
  reg.setInputCloud (cloud_xyz);
  reg.setInputNormals(cloud_normals);
  reg.setSmoothnessThreshold (10.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);
  reg.extract (clusters);

  return clusters;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
regrowPlaneExtraction(PointCloud::Ptr &cloud)
{
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  
  //***** Estimación de normales *********************************************//
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);
  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);
  ne.setKSearch(20);
  // ne.setRadiusSearch(0.015);
  ne.compute(*cloud_normals);


  //***** Segmentación basada en crecimiento de regiones *********************//
  pcl::RegionGrowing<PointT, pcl::Normal> reg;
  std::vector <pcl::PointIndices> clusters;
  reg.setMinClusterSize (100);
  reg.setMaxClusterSize (25000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (10);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (cloud_normals);
  reg.setSmoothnessThreshold (10.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);
  reg.extract (clusters);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();

  return colored_cloud;
}


pcl::PointIndices applyRANSAC(pcl::PointCloud<pcl::PointXYZLNormal>::Ptr &cloud, const bool optimizeCoefs,
                                        float distThreshold = 0.03, int maxIterations = 1000)
{
  pcl::SACSegmentation<pcl::PointXYZLNormal> ransac;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices inliers;

  ransac.setInputCloud(cloud);
  ransac.setOptimizeCoefficients(optimizeCoefs);
  ransac.setModelType(pcl::SACMODEL_PLANE);
  ransac.setMethodType(pcl::SAC_RANSAC);
  ransac.setMaxIterations(maxIterations);
  ransac.setDistanceThreshold(distThreshold);
  ransac.segment(inliers, *coefficients);

  return inliers;
}


float
computeRadius()
{

}

pcl::PointIndices::Ptr 
getBiggestCluster(PointCloud::Ptr &cloud_in,
                  std::vector<pcl::PointIndices> clusters_vector)
{
  PointCloud::Ptr out_cloud (new PointCloud);
  pcl::PointIndices::Ptr indices (new pcl::PointIndices);

  int max_size = 0;
  int max_clust_allocator;
  
  for (size_t i = 0; i < clusters_vector.size(); i++)
  {
    if(clusters_vector[i].indices.size() > max_size)
    {
      max_size = clusters_vector[i].indices.size();
      max_clust_allocator = i;
    }
  }

  *indices = clusters_vector[max_clust_allocator];

  return indices;
}


return_clouds
getGroundAndNoGroundCloud(PointCloud::Ptr &cloud_in,
                  std::vector<pcl::PointIndices> clusters_vector)
{
  return_clouds nubes_salida;
  pcl::PointIndices::Ptr indices (new pcl::PointIndices);
  
  int max_size = 500;
  
  for (size_t i = 0; i < clusters_vector.size(); i++)
  {
    if(clusters_vector[i].indices.size() > max_size)
    {
      for (int index : clusters_vector[i].indices)
        indices->indices.push_back(index);
    }
  }

  pcl::ExtractIndices<pcl::PointXYZLNormal> extract;
  extract.setInputCloud(cloud_in);
  extract.setIndices(indices);
  extract.setNegative(true);
  extract.filter(nubes_salida.no_ground);
  extract.setNegative(false);
  extract.filter(nubes_salida.ground);

  return nubes_salida;
}


void 
writeCloud(pcl::PointCloud<pcl::PointXYZLNormal>::Ptr &cloud_in, fs::path entry)
{
  pcl::PCDWriter pcd_writer;
  
  fs::path abs_file_path = fs::current_path().parent_path() / "pcd_xyzlabelnormal";
  if (!fs::exists(abs_file_path)) 
    fs::create_directory(abs_file_path);

  std::string filename = entry.stem().string() + ".pcd";

  abs_file_path = abs_file_path / filename;
  pcd_writer.write(abs_file_path.string(), *cloud_in, true);
}


// Plot clouds in two viewports
void 
visualizeClouds(PointCloud::Ptr &original_cloud, PointCloud::Ptr &filtered_cloud)
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


int main(int argc, char **argv)
{
  PointCloud::Ptr cloud_in (new PointCloud);
  PointCloud::Ptr cloud_out (new PointCloud);
  std::vector<pcl::PointIndices> ground_indices;
  return_clouds nubes_salida;
  PointCloud::Ptr ground_cloud (new PointCloud);
  PointCloud::Ptr no_ground_cloud (new PointCloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr regrow_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);


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
    }


  }
  // ONLY ONE CLOUD PASSED AS ARGUMENT IN CURRENT FOLDER
  else
  {
    fs::path entry = argv[1];
    cloud_in = readCloud(entry);
    ground_indices = computeClusters(cloud_in);
    regrow_cloud = regrowPlaneExtraction(cloud_in);
    nubes_salida = getGroundAndNoGroundCloud(cloud_in, ground_indices);
    *ground_cloud = nubes_salida.ground;
    *no_ground_cloud = nubes_salida.no_ground;

    pcl::PointIndices::Ptr idxes (new pcl::PointIndices);
    *idxes = applyRANSAC(cloud_in, false, 0.5, 1000);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(idxes);
    PointCloud::Ptr ransac_cloud (new PointCloud);
    extract.filter(*ransac_cloud);


    pcl::visualization::PCLVisualizer vis ("PCL Visualizer");
    int v1(0);
    int v2(0);

    //Define ViewPorts
    vis.createViewPort(0,0,0.5,1, v1);
    vis.createViewPort(0.5,0,1,1, v2);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> no_ground_color(no_ground_cloud, 100, 100, 100);
    vis.addPointCloud<PointT> (no_ground_cloud, no_ground_color, "no ground", v2);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> ground_color(ground_cloud, 0, 255, 0);
    vis.addPointCloud<PointT> (ground_cloud, ground_color, "ground", v2);
    // pcl::visualization::PointCloudColorHandlerCustom<PointT> ransac_color(ransac_cloud, 0, 0, 255);
    // vis.addPointCloud<PointT> (ransac_cloud, ransac_color, "ransac", v2);


    vis.addPointCloud<pcl::PointXYZRGB>(regrow_cloud, "regrow_cloud", v1);

    while(!vis.wasStopped())
      vis.spinOnce(100);

  }

  std::cout << GREEN << "COMPLETED!!" << RESET << std::endl;
  return 0;
}


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
typedef pcl::PointXYZLNormal PointT;
typedef pcl::PointCloud<PointT> PointCloud;
namespace fs = std::filesystem;


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
std::vector<pcl::PointIndices> computeClusters (PointCloud::Ptr &cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  pcl::copyPointCloud(*cloud, *cloud_xyz);


  // Estimación de normales
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_xyz);
  ne.setInputCloud(cloud_xyz);
  ne.setSearchMethod(tree);
  ne.setKSearch(20); // Por vecinos no existen normales NaN
  // ne.setRadiusSearch(0.05); // Por radio existiran puntos cuya normal sea NaN
  ne.compute(*cloud_normals);

  // Segmentación basada en crecimiento de regiones
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


////////////////////////////////////////////////////////////////////////////////
regrow 
regrowPlaneExtraction (PointCloud::Ptr &cloud)
{
  regrow return_values;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  
  //***** Estimación de normales *********************************************//
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);
  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);
  ne.setKSearch(20);
  // ne.setRadiusSearch(0.05);
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

  return_values.colored_cloud = reg.getColoredCloud();
  return_values.clusters = clusters;

  return return_values;
}


////////////////////////////////////////////////////////////////////////////////
PointCloud::Ptr
filterOutliersRegrow (PointCloud::Ptr &cloud, std::vector<pcl::PointIndices> clusters)
{
  pcl::PointIndices::Ptr indices (new pcl::PointIndices);
  for (size_t i = 0; i < clusters.size(); i++)
    for(auto index : clusters[i].indices)
      indices->indices.push_back(index);
  
  std::cout << "Clusters Indices Size: " << indices->indices.size() << std::endl;

  PointCloud::Ptr cloud_out (new PointCloud);
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(indices);
  extract.setNegative(false);
  extract.filter(*cloud_out);

  return cloud_out;
}


////////////////////////////////////////////////////////////////////////////////
pcl::PointIndices::Ptr 
applyRANSAC (pcl::PointCloud<pcl::PointXYZLNormal>::Ptr &cloud, const bool optimizeCoefs,
            float distThreshold = 0.03, int maxIterations = 1000)
{
  pcl::SACSegmentation<pcl::PointXYZLNormal> ransac;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers;

  ransac.setInputCloud(cloud);
  ransac.setOptimizeCoefficients(optimizeCoefs);
  ransac.setModelType(pcl::SACMODEL_PLANE);
  ransac.setMethodType(pcl::SAC_RANSAC);
  ransac.setMaxIterations(maxIterations);
  ransac.setDistanceThreshold(distThreshold);
  ransac.segment(*inliers, *coefficients);

  return inliers;
}


////////////////////////////////////////////////////////////////////////////////
pcl::PointIndices::Ptr 
getBiggestCluster (PointCloud::Ptr &cloud_in,
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


////////////////////////////////////////////////////////////////////////////////
filterGroundClouds
filterGroundByVolume (PointCloud::Ptr &cloud_in, std::vector<pcl::PointIndices> clusters_vector)
{
  filterGroundClouds nubes_salida;
  pcl::PointIndices::Ptr indices (new pcl::PointIndices);
  pcl::PointIndices::Ptr tmp_indices (new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::copyPointCloud(*cloud_in, *cloud_xyz);

  float min_volumen = 0.1;
  
  for (size_t i = 0; i < clusters_vector.size(); i++)
  {
    *tmp_indices = clusters_vector[i];

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_xyz);
    extract.setIndices(tmp_indices);
    extract.setNegative(false);
    extract.filter(*tmp_cloud);

    arvc::arvcBoundBox bound_box;
    bound_box = arvc::computeBoundingBox(tmp_cloud);
    float bb_volumen = bound_box.width * bound_box.height * bound_box.depth;

    if(bb_volumen > min_volumen)
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


/**
 * @brief Remove points with less than minNeighbors inside a give radius
 * 
 * @param cloud PointCloud to remove
 * @param radius Radius inside to look for points
 * @param minNeighbors Number on minimum Neighbors to not remove a point
 * @return PointCloud::Ptr Return a PointCloud without low neighbor points
 */
PointCloud::Ptr
radiusOutlierRemoval (PointCloud::Ptr &cloud, float radius, int minNeighbors)
{
  PointCloud::Ptr cloud_out (new PointCloud);
  pcl::RadiusOutlierRemoval<PointT> radius_removal;
  radius_removal.setInputCloud(cloud);
  radius_removal.setRadiusSearch(radius);
  radius_removal.setMinNeighborsInRadius(minNeighbors);
  radius_removal.filter(*cloud_out);
  std::cout << "Removed " << (cloud->points.size() - cloud_out->points.size()) << "points" << std::endl;

  return cloud_out;
}


////////////////////////////////////////////////////////////////////////////////
void 
writeCloud (pcl::PointCloud<pcl::PointXYZLNormal>::Ptr &cloud_in, fs::path entry)
{
  pcl::PCDWriter pcd_writer;
  
  fs::path abs_file_path = fs::current_path().parent_path() / "pcd_xyzlabelnormal";
  if (!fs::exists(abs_file_path)) 
    fs::create_directory(abs_file_path);

  std::string filename = entry.stem().string() + ".pcd";

  abs_file_path = abs_file_path / filename;
  pcd_writer.write(abs_file_path.string(), *cloud_in, true);
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
  auto start = std::chrono::high_resolution_clock::now();
  PointCloud::Ptr cloud_in (new PointCloud);
  PointCloud::Ptr regrow_filtered (new PointCloud);
  PointCloud::Ptr ground_cloud (new PointCloud);
  PointCloud::Ptr no_ground_cloud (new PointCloud);

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

    PointCloud::Ptr cloud_filtered = radiusOutlierRemoval(cloud_in, 0.1, 10);

    regrow regrow_data = regrowPlaneExtraction(cloud_filtered);

    regrow_filtered = filterOutliersRegrow(cloud_filtered, regrow_data.clusters);

    regrow regrow_data2 =regrowPlaneExtraction(regrow_filtered);

    filterGroundClouds segment_clouds = filterGroundByVolume(regrow_filtered, regrow_data2.clusters);

    *ground_cloud = segment_clouds.ground;
    *no_ground_cloud = segment_clouds.no_ground;

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    std::cout << "Computation Time: " << duration.count() << " ms" << std::endl;



  //////////////////////////////////////////////////////////////////////////////
  // VISUALIZATION

    pcl::visualization::PCLVisualizer vis ("PCL Visualizer");
    pcl::visualization::PCLVisualizer vis2 ("vis2");
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
    // vis.createViewPort(0,0.5,0.5,1, v1);
    // vis.createViewPort(0.5,0.5,1,1, v2);
    // vis.createViewPort(0,0,0.5,0.5, v3);
    // vis.createViewPort(0.5,0,1,0.5, v4);

    vis.addPointCloud<PointT>(cloud_in, "original_cloud", v1);
    // vis.addPointCloud<pcl::PointXYZRGB>(regrow_data.colored_cloud, "regrow_cloud", v2);
    // vis.addPointCloud<pcl::PointXYZRGB>(regrow_data2.colored_cloud, "regrow_cloud2", v3);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> no_ground_color(no_ground_cloud, 0, 255, 0);
    // vis.addPointCloud<PointT> (no_ground_cloud, no_ground_color, "no ground", v2);
    vis2.addPointCloud<PointT> (no_ground_cloud, no_ground_color, "no ground");


    while(!vis.wasStopped())
      vis.spinOnce(100);

  //////////////////////////////////////////////////////////////////////////////
  }

  std::cout << GREEN << "COMPLETED!!" << RESET << std::endl;
  return 0;
}


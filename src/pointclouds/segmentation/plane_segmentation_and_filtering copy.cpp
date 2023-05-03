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

// #define RESET   "\033[0m"
// #define RED     "\033[31m"
// #define GREEN   "\033[32m"  
// #define YELLOW  "\033[33m"
// #define BLUE    "\033[34m"

//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
namespace fs = std::filesystem;
using namespace std;



////////////////////////////////////////////////////////////////////////////////
struct filterGroundClouds
{
  pcl::PointCloud<pcl::PointXYZ> ground;
  pcl::PointCloud<pcl::PointXYZ> no_ground;
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
std::vector<pcl::PointIndices> regrow_segmentation (PointCloud::Ptr &cloud)
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
  ne.setKSearch(30); // Por vecinos no existen normales NaN
  // ne.setRadiusSearch(0.05); // Por radio existiran puntos cuya normal sea NaN
  ne.compute(*cloud_normals);

  // Segmentación basada en crecimiento de regiones
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  std::vector <pcl::PointIndices> clusters;
  reg.setMinClusterSize (100);
  reg.setMaxClusterSize (25000);
  reg.setSearchMethod (tree);
  // reg.setSmoothModeFlag(false);
  reg.setCurvatureTestFlag(true);
  reg.setResidualThreshold(false);
  reg.setCurvatureThreshold(1);
  reg.setNumberOfNeighbours (10);
  reg.setInputCloud (cloud_xyz);
  reg.setInputNormals (cloud_normals);
  reg.setSmoothnessThreshold (10.0 / 180.0 * M_PI);
  reg.extract (clusters);

  return clusters;
}


/**
 * @brief Returns a PointCloud containing only the indices passed as a parameter
 * 
 * @param cloud Initial cloud
 * @param indx_vec Indices of points that should extract from cloud_in
 * @return PointCloud::Ptr 
 */
PointCloud::Ptr
extract_indices (PointCloud::Ptr &cloud_in, std::vector<pcl::PointIndices> indx_vec, bool negative = false)
{
  pcl::PointIndices::Ptr indices (new pcl::PointIndices);

  for (size_t i = 0; i < indx_vec.size(); i++)
    for(auto index : indx_vec[i].indices)
      indices->indices.push_back(index);
  
  cout << "Tamaño del vector con bucle for: " << indices->indices.size() << endl;

  PointCloud::Ptr cloud_out (new PointCloud);
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_in);
  extract.setIndices(indices);
  extract.setNegative(negative);
  extract.filter(*cloud_out);

  return cloud_out;
}

/**
 * @brief Returns a PointCloud containing only the indices passed as a parameter
 * 
 * @param cloud_in Initial cloud
 * @param indices Indices of points that should extract from cloud_in
 * @return PointCloud::Ptr 
 */
PointCloud::Ptr 
extract_indices (PointCloud::Ptr &cloud_in, pcl::PointIndices::Ptr &indices, bool negative = false)
{
  PointCloud::Ptr cloud_out (new PointCloud);
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_in);
  extract.setIndices(indices);
  extract.setNegative(negative);
  extract.filter(*cloud_out);

  return cloud_out;
}

////////////////////////////////////////////////////////////////////////////////
pcl::ModelCoefficients::Ptr 
compute_planar_ransac (PointCloud::Ptr &cloud, const bool optimizeCoefs,
            float distThreshold = 0.03, int maxIterations = 1000)
{
  pcl::SACSegmentation<PointT> ransac;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  ransac.setInputCloud(cloud);
  ransac.setOptimizeCoefficients(optimizeCoefs);
  ransac.setModelType(pcl::SACMODEL_PLANE);
  ransac.setMethodType(pcl::SAC_RANSAC);
  ransac.setMaxIterations(maxIterations);
  ransac.setDistanceThreshold(distThreshold);
  ransac.segment(*inliers, *coefficients);

  return coefficients;
}


pcl::PointIndices::Ptr
get_point_near_plane(PointCloud::Ptr &cloud_in, 
                   pcl::ModelCoefficients::Ptr &coefs, float distThreshold = 0.5)
{
  pcl::PointIndices::Ptr indices (new pcl::PointIndices);
  Eigen::Vector4f coefficients(coefs->values.data());
  pcl::PointXYZ point;

  for (size_t i; i < cloud_in->points.size(); i++)
  {
    point = cloud_in->points[i];
    if (pcl::pointToPlaneDistance(point, coefficients) <= distThreshold)
      indices->indices.push_back(i);
  }
  
  return indices;
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

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud_in);
  extract.setIndices(indices);
  extract.setNegative(true);
  extract.filter(nubes_salida.no_ground);
  extract.setNegative(false);
  extract.filter(nubes_salida.ground);

  return nubes_salida;
}

/**
 * @brief Remove points with less than minNeighbors inside a give radius
 * 
 * @param cloud PointCloud to apply the filter
 * @param radius Search sphere radius
 * @param minNeighbors Minimum num of Neighbors to consider a point an inlier
 * @return PointCloud::Ptr Return a PointCloud without low neighbor points
 */
PointCloud::Ptr
radius_outlier_removal (PointCloud::Ptr &cloud, float radius, int minNeighbors)
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


/**
 * @brief Returns a Voxelized PointCloud
 * 
 * @param cloud_in 
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr
 */
PointCloud::Ptr
voxel_filter(PointCloud::Ptr &cloud_in, float leafSize = 0.1)
{
  PointCloud::Ptr cloud_out (new PointCloud);
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(cloud_in);
  sor.setLeafSize(leafSize, leafSize, leafSize);
  sor.filter(*cloud_out);

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

void 
visualizeCloud (PointCloud::Ptr &original_cloud)
{
  pcl::visualization::PCLVisualizer vis("PCL_Visualizer");

  int v1(0);
  vis.addPointCloud<PointT> (original_cloud, "Original", v1);

  while(!vis.wasStopped())
    vis.spinOnce(100);
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  auto start = std::chrono::high_resolution_clock::now();
  PointCloud::Ptr cloud_in (new PointCloud);
  PointCloud::Ptr working_cloud (new PointCloud);
  PointCloud::Ptr ground_cloud (new PointCloud);
  PointCloud::Ptr no_ground_cloud (new PointCloud);

  

  vector<pcl::PointIndices> clusters;
  pcl::PointIndices::Ptr sac_indices (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr sac_coefs (new pcl::ModelCoefficients);
  PointCloud::Ptr clusters_cloud (new PointCloud);

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

    working_cloud = radius_outlier_removal(cloud_in, 0.1, 5);
    clusters = regrow_segmentation(working_cloud);
    working_cloud = extract_indices(working_cloud, clusters, false);
    working_cloud = voxel_filter(working_cloud, 0.1);
    sac_coefs = compute_planar_ransac(working_cloud, true, 0.5, 1000);
    
    working_cloud = extract_indices(working_cloud, sac_indices, true);
    visualizeCloud(working_cloud);


    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    std::cout << "Computation Time: " << duration.count() << " ms" << std::endl;

  // //////////////////////////////////////////////////////////////////////////////
  // // VISUALIZATION

  //   pcl::visualization::PCLVisualizer vis ("PCL Visualizer");
  //   pcl::visualization::PCLVisualizer vis2 ("vis2");
  //   // Define ViewPorts
  //   int v1(0);
  //   int v2(0);
  //   int v3(0);
  //   int v4(0);

  //   // 2 HORIZONTAL VIEWPORTS
  //   // vis.createViewPort(0,0,0.5,1, v1);
  //   // vis.createViewPort(0.5,0,1,1, v2);

  //   // 3 HORIZONTAL VIEWPORTS
  //   // vis.createViewPort(0,0,0.33,1, v1);
  //   // vis.createViewPort(0.33,0,0.66,1, v2);
  //   // vis.createViewPort(0.66,0,1,1, v3);

  //   // 4 DISTRIBUTED VIEWPORTS
  //   // vis.createViewPort(0,0.5,0.5,1, v1);
  //   // vis.createViewPort(0.5,0.5,1,1, v2);
  //   // vis.createViewPort(0,0,0.5,0.5, v3);
  //   // vis.createViewPort(0.5,0,1,0.5, v4);

  //   vis.addPointCloud<PointT>(cloud_in, "original_cloud", v1);
  //   // vis.addPointCloud<pcl::PointXYZRGB>(regrow_data.colored_cloud, "regrow_cloud", v2);
  //   // vis.addPointCloud<pcl::PointXYZRGB>(regrow_data2.colored_cloud, "regrow_cloud2", v3);

  //   pcl::visualization::PointCloudColorHandlerCustom<PointT> no_ground_color(no_ground_cloud, 0, 255, 0);
  //   // vis.addPointCloud<PointT> (no_ground_cloud, no_ground_color, "no ground", v2);
  //   vis2.addPointCloud<PointT> (no_ground_cloud, no_ground_color, "no ground");


  //   while(!vis.wasStopped())
  //     vis.spinOnce(100);

  //////////////////////////////////////////////////////////////////////////////
  }

  std::cout << GREEN << "COMPLETED!!" << RESET << std::endl;
  return 0;
}


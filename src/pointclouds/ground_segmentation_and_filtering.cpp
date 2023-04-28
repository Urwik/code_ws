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

namespace fs = std::filesystem;
using namespace std;


class GroundSeg
{
  typedef pcl::PointXYZ PointT;
  typedef pcl::PointCloud<PointT> PointCloud;
  typedef pcl::PointXYZI PointI;
  typedef pcl::PointCloud<PointI> PointCloudI;

  struct metrics
  {
    float precision = 0.0;
    float recall = 0.0;
    float accuracy = 0.0;
  };

  struct conf_matrix
  {
    int TP = 0;
    int FP = 0;
    int TN = 0;
    int FN = 0;
  };


private:
  pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud_intensity;


  /**
   * @brief Get the Ground Truth object
   * 
   */
  void
  _getGroundTruthIndices()
  {
    pcl::PassThrough<pcl::PointXYZI> pt;
    pt.setInputCloud(this->_cloud_intensity);
    pt.setFilterFieldName("intensity");
    pt.setFilterLimits(0, 0);
    pt.setNegative(false);
    pt.filter(*this->gt_ground_indices);
    pt.setNegative(true);
    pt.filter(*this->gt_truss_indices);
  }


public:
  fs::path path;
  PointCloud::Ptr cloud_in;

  PointCloud::Ptr ground_cloud;
  PointCloud::Ptr truss_cloud;
  PointCloud::Ptr current_cloud;
  PointCloud::Ptr gt_ground_cloud;
  PointCloud::Ptr gt_truss_cloud;

  pcl::IndicesPtr ground_indices;
  pcl::IndicesPtr truss_indices;
  pcl::IndicesPtr current_indices;
  pcl::IndicesPtr gt_ground_indices;
  pcl::IndicesPtr gt_truss_indices;

  boost::shared_ptr<std::vector<pcl::PointIndices>> regrow_clusters;


  // Constructor
  GroundSeg(){
    cloud_in = PointCloud::Ptr (new PointCloud);

    ground_cloud = PointCloud::Ptr (new PointCloud);
    truss_cloud = PointCloud::Ptr (new PointCloud);
    current_cloud = PointCloud::Ptr (new PointCloud);
    gt_ground_cloud = PointCloud::Ptr (new PointCloud);
    gt_truss_cloud = PointCloud::Ptr (new PointCloud);
    
    ground_indices = pcl::IndicesPtr (new pcl::Indices);
    truss_indices = pcl::IndicesPtr (new pcl::Indices);
    current_indices = pcl::IndicesPtr (new pcl::Indices);
    gt_ground_indices = pcl::IndicesPtr (new pcl::Indices);
    gt_truss_indices = pcl::IndicesPtr (new pcl::Indices);

    regrow_clusters = boost::shared_ptr<std::vector<pcl::PointIndices>> (new std::vector<pcl::PointIndices>);

  }
  
  
  // Destructor
  ~GroundSeg(){
    cloud_in->clear();
    
    current_cloud->clear();
    ground_cloud->clear();
    truss_cloud->clear();
    
    ground_indices->clear();
    truss_indices->clear();
    current_indices->clear();


  }

  /**
   * @brief Lee una nube de puntos en formato .pcd o .ply
   * 
   * @param path Ruta de la nube de puntos
   * @return PointCloudI::Ptr 
   */
  int
  readCloud ()
  {
    map<string, int> ext_map = {{".pcd", 0}, {".ply", 1}};

    switch (ext_map[this->path.extension().string()])
    {
      case 0: {
        pcl::PCDReader pcd_reader;
        pcd_reader.read(path.string(), *this->_cloud_intensity);
        break;
      }
      case 1: {
        pcl::PLYReader ply_reader;
        ply_reader.read(path.string(), *this->_cloud_intensity);
        break;
      }
      default: {
        std::cout << "Format not compatible, it should be .pcd or .ply" << std::endl;
        return -1;
        break;
      }
    }

    // Convertir a XYZ
    pcl::copyPointCloud(*this->_cloud_intensity, *this->cloud_in);

    return 0;
  }


  /**
   * @brief Realiza agrupaciones de puntos en función de sus normales
   * 
   * @param cloud  Nube de entrada
   * @return std::vector<pcl::PointIndices> Vector con los indices pertenecientes 
   * a cada agrupación 
   */
  void
  regrow_segmentation ()
  {
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Estimación de normales
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(this->cloud_in);
    ne.setInputCloud(this->cloud_in);
    ne.setSearchMethod(tree);
    ne.setKSearch(30); // Por vecinos no existen normales NaN
    // ne.setRadiusSearch(0.05); // Por radio existiran puntos cuya normal sea NaN
    ne.compute(*cloud_normals);

    // Segmentación basada en crecimiento de regiones
    pcl::RegionGrowing<PointT, pcl::Normal> reg;
    reg.setMinClusterSize (100);
    reg.setMaxClusterSize (25000);
    reg.setSearchMethod (tree);
    reg.setSmoothModeFlag(false);
    reg.setCurvatureTestFlag(true);
    reg.setResidualThreshold(false);
    reg.setCurvatureThreshold(1);
    reg.setNumberOfNeighbours (10);
    reg.setInputCloud (this->cloud_in);
    reg.setInputNormals (cloud_normals);
    reg.setSmoothnessThreshold (10.0 / 180.0 * M_PI);
    reg.extract (*this->regrow_clusters);

    // // Uncomment to visualize cloud
    // pcl::visualization::PCLVisualizer vis ("PCL Visualizer");
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    // color_cloud = reg.getColoredCloud();
    // vis.addPointCloud<pcl::PointXYZRGB>(color_cloud);

    // while (!vis.wasStopped())
    //   vis.spinOnce();

    // return clusters;
  }


  /**
   * @brief Filtra la nube de puntos en función de los índices pasados como parámetro
   * 
   * @param cloud Nube de entrada
   * @param indices_vec Vector con los índices de los puntos que se quieren extraer
   * @param negative Si es true, se extraen los puntos que no están en indx_vec
   */
  void
  extract_indices(PointCloud::Ptr &cloud, std::vector<pcl::PointIndices> indices_vec, bool negative = false)
  {
    pcl::PointIndices::Ptr indices (new pcl::PointIndices);

    for (size_t i = 0; i < indices_vec.size(); i++)
      for(auto index : indices_vec[i].indices)
        indices->indices.push_back(index);
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.setNegative(negative);
    extract.filter(*this->current_cloud);
    extract.filter(*this->current_indices);
  }


  /**
   * @brief Filtra la nube de puntos en función de los índices pasados como parámetro
   * 
   * @param cloud Nube de entrada
   * @param indices Indices de los puntos que se quieren extraer
   * @param negative Si es true, se extraen los puntos que no están en indx_vec
   */
  void
  extract_indices (PointCloud::Ptr &cloud, pcl::IndicesPtr &indices, bool negative = false)
  {
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.setNegative(negative);
    extract.filter(*this->current_cloud);
    extract.filter(*this->current_indices);
  }

  /**
   * @brief Filtra la nube de puntos en función de los índices pasados como parámetro
   * 
   * @param cloud 
   * @param optimizeCoefs 
   * @param distThreshold 
   * @param maxIterations 
   * @return pcl::ModelCoefficients::Ptr 
   */
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


  /**
   * @brief Filtra la nube de puntos en función de los índices pasados como parámetro
   * 
   * @param cloud 
   * @param coefs 
   * @param distThreshold 
   * @return pcl::PointIndices::Ptr 
   */
  pcl::PointIndices::Ptr
  get_points_near_plane(PointCloud::Ptr &cloud_in, 
                    pcl::ModelCoefficients::Ptr &coefs, float distThreshold = 0.5)
  {
    pcl::PointIndices::Ptr indices (new pcl::PointIndices);
    Eigen::Vector4f coefficients(coefs->values.data());
    pcl::PointXYZ point;

    for (size_t i=0; i < cloud_in->points.size(); i++)
    {
      point = cloud_in->points[i];
      float distance = pcl::pointToPlaneDistance(point, coefficients);
      if (pcl::pointToPlaneDistance(point, coefficients) <= distThreshold)
        indices->indices.push_back(i);
    }
    
    return indices;
  }


  /**
   * @brief Computes the eigenvalues of a PointCloud
   * 
   * @param cloud_in 
   * @return Eigen::Vector3f 
   */
  Eigen::Vector3f
  compute_eigenvalues(PointCloud::Ptr &cloud_in)
  {
    Eigen::Vector4f xyz_centroid;
    pcl::compute3DCentroid(*cloud_in, xyz_centroid);

    Eigen::Matrix3f covariance_matrix;
    pcl::computeCovarianceMatrixNormalized (*cloud_in, xyz_centroid, covariance_matrix); 
    // pcl::computeCovarianceMatrix (*cloud_in, xyz_centroid, covariance_matrix); 


    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix, Eigen::ComputeEigenvectors);
    Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();

    return eigenValuesPCA;
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


  /**
   * @brief Returns a Voxelized PointCloud
   * 
   * @param cloud_in 
   * @return pcl::PointCloud<pcl::PointXYZ>::Ptr
   */
  metrics
  compute_metrics(PointCloud::Ptr &gt_truss, PointCloud::Ptr &gt_ground,  PointCloud::Ptr &truss_cloud, PointCloud::Ptr &ground_cloud)
  {
    metrics metrics_;
    conf_matrix conf_matrix_;
    PointCloud tmp_cloud;
    // tmp_cloud = *truss_cloud - *gt_ground;

    conf_matrix_.TP = truss_cloud->size() - tmp_cloud.size();

    return metrics_;
  }

  /**
   * @brief Returns a Voxelized PointCloud
   * 
   * @param cloud_in 
   * @return pcl::PointCloud<pcl::PointXYZ>::Ptr
   */
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


  /**
   * @brief Returns a Voxelized PointCloud
   * 
   * @param cloud_in 
   * @return pcl::PointCloud<pcl::PointXYZ>::Ptr
   */
  void 
  visualizeCloud (PointCloud::Ptr &original_cloud)
  {
    pcl::visualization::PCLVisualizer vis("PCL_Visualizer");

    int v1(0);
    vis.addPointCloud<PointT> (original_cloud, "Original", v1);

    while(!vis.wasStopped())
      vis.spinOnce(100);

    vis.close();
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
  radius_outlier_removal (PointCloud::Ptr &cloud_in, float radius, int minNeighbors)
  {
    PointCloud::Ptr cloud_out (new PointCloud);
    pcl::RadiusOutlierRemoval<PointT> radius_removal;
    radius_removal.setInputCloud(cloud_in);
    radius_removal.setRadiusSearch(radius);
    radius_removal.setMinNeighborsInRadius(minNeighbors);
    radius_removal.filter(*cloud_out);

    return cloud_out;
  }

};


////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  std::cout << GREEN << "Running your code..." << RESET << std::endl;
  auto start = std::chrono::high_resolution_clock::now();

  GroundSeg ground_seg;

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
    ground_seg.path = entry;
    ground_seg.radius_outlier_removal(ground_seg.cloud_in, 0.1, 5);
    ground_seg.voxel_filter(ground_seg.cloud_in, 0.05);
    pcl::ModelCoefficients::Ptr asdf (new pcl::ModelCoefficients);
    asdf = ground_seg.compute_planar_ransac(ground_seg.cloud_in, 0.1, 1000, 0.1);
    ground_seg.get_points_near_plane(ground_seg.cloud_in, asdf, 0.5);
  }


  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

  std::cout << "Computation Time: " << duration.count() << " ms" << std::endl;
  std::cout << GREEN << "Code end!!" << RESET << std::endl;
  return 0;
}

void
vis_multi_viewports()
{
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

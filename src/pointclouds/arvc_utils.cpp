// #pragma once
// cpp
#include <iostream>

// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_representation.h>
#include <pcl/recognition/linemod/line_rgbd.h>

  // PCL FILTERS
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>


#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"  
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"


namespace arvc
{

//////////////////////////////////////////////////////////////////////////////
  pcl::PointCloud<pcl::PointXYZLNormal>::Ptr
  readCloud(std::filesystem::path path)
  {
    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZLNormal>);

    std::string file_ext = path.extension();
    if (file_ext == ".pcd") {
      pcl::PCDReader pcd_reader;
      pcd_reader.read(path.string(), *cloud);
    }
    else if (file_ext == ".ply") {
      pcl::PLYReader ply_reader;
      ply_reader.read(path.string(), *cloud);
    }
    else
      std::cout << "Format not compatible, it should be .pcd or .ply" << std::endl;

    return cloud;
  }




////////////////////////////////////////////////////////////////////////////////  
  pcl::PointCloud<pcl::PointXYZL>::Ptr
  parseIntensityToLabel(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in)
  {
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZL>);
    cloud_out->points.resize(cloud_in->points.size());

    for (size_t i = 0; i < cloud_in->size(); i++)
    {
      cloud_out->points[i].x = cloud_in->points[i].x;
      cloud_out->points[i].y = cloud_in->points[i].y;
      cloud_out->points[i].z = cloud_in->points[i].z;
      cloud_out->points[i].label = (uint32_t) cloud_in->points[i].intensity;
    }

    return cloud_out;  
  }

  ////////////////////////////////////////////////////////////////////////////////
  pcl::PointCloud<pcl::PointXYZ>::Ptr 
  extractIndices(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
    pcl::PointIndices::Ptr &indices)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(indices);
    extract.filter(*cloud_out);

    return cloud_out;
  }

////////////////////////////////////////////////////////////////////////////////  
  pcl::PointCloud<pcl::PointXYZ>::Ptr
  parseXYZLNormalToXYZ(pcl::PointCloud<pcl::PointXYZLNormal>::Ptr &cloud_in)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_out->points.resize(cloud_in->points.size());

    for (size_t i = 0; i < cloud_in->size(); i++)
    {
      cloud_out->points[i].x = cloud_in->points[i].x;
      cloud_out->points[i].y = cloud_in->points[i].y;
      cloud_out->points[i].z = cloud_in->points[i].z;
    }

    return cloud_out;  
  }


////////////////////////////////////////////////////////////////////////////////
  pcl::PointIndices::Ptr 
  computePlaneInliersRANSAC(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const bool optimizeCoefs,
    float distThreshold = 0.03,
    int maxIterations = 1000)
  {
    pcl::SACSegmentation<pcl::PointXYZ> ransac;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

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
  Eigen::Vector4f 
  computePlaneCoefsRANSAC(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const bool optimizeCoefs,
    float distThreshold = 0.03, 
    int maxIterations = 1000)
  {
    pcl::SACSegmentation<pcl::PointXYZ> ransac;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    ransac.setInputCloud(cloud);
    ransac.setOptimizeCoefficients(optimizeCoefs);
    ransac.setModelType(pcl::SACMODEL_PLANE);
    ransac.setMethodType(pcl::SAC_RANSAC);
    ransac.setMaxIterations(maxIterations);
    ransac.setDistanceThreshold(distThreshold);
    ransac.segment(*inliers, *coefficients);


    Eigen::Vector4f plane_coefs;
    for (size_t i = 0; i < 4; i++)
      plane_coefs[i] = coefficients->values[i];
    

    return plane_coefs;
  }


////////////////////////////////////////////////////////////////////////////////
  struct arvcBoundBox
  {
    pcl::PointXYZ centroid;
    float width;
    float height;
    float depth;
  };

  arvcBoundBox 
  computeBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
  {
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloud, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();

    /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
    /// the signs are different and the box doesn't get correctly oriented in some cases.
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  
                                                                                    

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZ minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);

    arvcBoundBox bounding_box;

    bounding_box.centroid.x = pcaCentroid(0);
    bounding_box.centroid.y = pcaCentroid(1);
    bounding_box.centroid.z = pcaCentroid(2);

    bounding_box.width = std::abs(maxPoint.x - minPoint.x);
    bounding_box.height = std::abs(maxPoint.y - minPoint.y);
    bounding_box.depth = std::abs(maxPoint.z - minPoint.z);

    return bounding_box;
  }


////////////////////////////////////////////////////////////////////////////////
  /**
   * @brief Calcula la roughness dado un punto.
   * 
   * @param cloud_in Nube de entrada
   * @param index Indice del punto a evaluar.
   * @return float 
   */
  float
  computeRoughness(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
    int index)
  {
    float point_roughness;
    pcl::PointXYZ target_point = cloud_in->points[index];

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_in);
    int K = 10;

    pcl::PointIndices::Ptr indicesKnearest (new pcl::PointIndices);
    std::vector<float> pointNKNSquaredDistance(K);

    if( kdtree.nearestKSearch (*cloud_in, index, K, indicesKnearest->indices, pointNKNSquaredDistance) > 0 )
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr ngbh_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      Eigen::Vector4f plane_coefs;
      ngbh_cloud = extractIndices(cloud_in, indicesKnearest);
      plane_coefs = computePlaneCoefsRANSAC(ngbh_cloud, true, 0.03, 1000);

      point_roughness = pcl::pointToPlaneDistance(target_point, plane_coefs);
    }
    else
      point_roughness = 100;

    return point_roughness;
  }


////////////////////////////////////////////////////////////////////////////////
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
  getColoredCloudFromRoughness(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
    std::vector<float> roughness)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_out->resize(cloud_in->size());

    for (size_t i = 0; i < cloud_in->size(); i++)
    {
      cloud_out->points[i].x = cloud_in->points[i].x;
      cloud_out->points[i].y = cloud_in->points[i].y;
      cloud_out->points[i].z = cloud_in->points[i].z;
    }
    
  }

}

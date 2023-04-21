// cpp
#include <iostream>
#include <algorithm>
#include <filesystem>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

  }


////////////////////////////////////////////////////////////////////////////////  
  pcl::PointCloud<pcl::PointXYZL>::Ptr
  parseIntensityToLabel(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in)
  {

  }



////////////////////////////////////////////////////////////////////////////////
  pcl::PointCloud<pcl::PointXYZ>::Ptr 
  extractIndices(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
    pcl::PointIndices::Ptr &indices)
  {

  }



////////////////////////////////////////////////////////////////////////////////
  pcl::PointCloud<pcl::PointXYZLNormal>::Ptr
  voxelFilter(pcl::PointCloud<pcl::PointXYZLNormal>::Ptr &cloud_in)
  {

  }


////////////////////////////////////////////////////////////////////////////////  
  pcl::PointCloud<pcl::PointXYZ>::Ptr
  parseXYZLNormalToXYZ(pcl::PointCloud<pcl::PointXYZLNormal>::Ptr &cloud_in)
  {

  }



////////////////////////////////////////////////////////////////////////////////
  pcl::PointIndices::Ptr 
  computePlaneInliersRANSAC(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const bool optimizeCoefs,
    float distThreshold = 0.03,
    int maxIterations = 1000)
  {

  }



////////////////////////////////////////////////////////////////////////////////
  Eigen::Vector4f 
  computePlaneCoefsRANSAC(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const bool optimizeCoefs,
    float distThreshold = 0.03, 
    int maxIterations = 1000)
  {

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

  }


////////////////////////////////////////////////////////////////////////////////

//input: ratio is between 0.0 to 1.0
//output: rgb color
struct myRgb{
  int r;
  int g;
  int b;
};
myRgb rgb(double ratio)
{

}



////////////////////////////////////////////////////////////////////////////////
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
  getColoredCloudFromRoughness(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
    std::vector<float> roughness)
  {
  }

}

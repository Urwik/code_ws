// cpp
#include <iostream>
#include <algorithm>
#include <filesystem>
#include <istream>
#include <algorithm>

// PCL
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Eigen>


//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;
// pcl::visualization::PCLVisualizer::Ptr pclVisualizer (new pcl::visualization::PCLVisualizer ("PCL Visualizer"));

// ************************************************************************** //
namespace fs = boost::filesystem;

////////////////////////////////////////////////////////////////////////////////
template <typename Derived>
std::string get_shape(const Eigen::EigenBase<Derived>& x)
{
    std::ostringstream oss;
    oss  << "[" << x.rows() << ", " << x.cols() << "]";
    return oss.str();
}

struct myPoint
{
  float x;
  float y;
  float z;
  float intensity;
}; 



Eigen::MatrixXf pclCloud2Eigen(PointCloud cloud)
{
  int n_points = cloud.points.size();

  Eigen::MatrixXf m_cloud(n_points, 4);
  Eigen::MatrixXf my_row(1, 4);

  int indx=0;
  for (auto point : cloud)
  {
    std::cout << indx << std::endl;
    my_row << point.x , point.y, point.z, point.intensity;
    m_cloud.row(indx) = my_row;
    indx++;
  }

  return m_cloud;
}


std::vector<myPoint> pclCloud2Vector(PointCloud cloud)
{
  std::vector<myPoint> my_cloud;
  myPoint tmp_point;

  int indx=0;

  for (auto point : cloud)
  {
    tmp_point.x = point.x;
    tmp_point.y = point.y;
    tmp_point.z = point.z;
    tmp_point.intensity = point.intensity;
    my_cloud.push_back(tmp_point);
    indx++;
  }

  return my_cloud;
}

int main(int argc, char **argv)
{
  // Get handlres for source and target cloud data /////////////////////////////
  fs::path current_path = fs::current_path();
  pcl::PCDReader pcd_reader;
  PointCloud::Ptr cloud (new PointCloud);

  pcd_reader.read("/home/arvc/arvc_saved_cloud.pcd", *cloud);

  Eigen::MatrixXf m_cloud = pclCloud2Eigen(*cloud);
  Eigen::VectorXf intensity = m_cloud.col(3);

  std::vector<myPoint> my_cloud = pclCloud2Vector(*cloud);

  std::vector<float> intensity_vector;

  for (myPoint point : my_cloud)
  {
    intensity_vector.push_back(point.intensity);
  }

  // std::vector<int>::iterator asdf = std::find(intensity_vector.begin(), intensity_vector.end(), 5);  
  return 0;
}

// cpp
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


//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
// ************************************************************************** //
namespace fs = boost::filesystem;

////////////////////////////////////////////////////////////////////////////////


int main(int argc, char **argv)
{
  // Get handlres for source and target cloud data /////////////////////////////
  fs::path current_path = fs::current_path();
  pcl::PCDReader pcd_reader;
  pcl::PCDWriter pcd_writer;


  // if (argc <= 1)
  //   std::cout << "NEED TO SPECIFY THE FILE" << std::endl;
  
  // std::string filename = argv[1];
  

  // fs::path abs_file_path;
  // abs_file_path = current_path.c_str() + '/' + filename;

  PointCloud::Ptr cloud (new PointCloud);
  pcd_reader.read("/media/arvc/data/experiments/ouster/simulated/73_pcds/results/map_Generalized.pcd", *cloud);

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>); 

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.1);

  // Compute the features
  ne.compute(*cloud_normals);
  pcl::concatenateFields(*cloud, *cloud_normals, *cloud_with_normals); 
  pcd_writer.write("/media/arvc/data/experiments/ouster/simulated/73_pcds/results/map_with_normals.pcd", *cloud_with_normals, false);


  return 0;
}




#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "arvc_utils.hpp"
#include <pcl/segmentation/extract_clusters.h>



int main(int argc, char** argv)
{
  PointCloud::Ptr input(new PointCloud);
  input = arvc::readCloud("../examples/example_cloud.pcd");

  PointCloud::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
  vector<pcl::PointIndices> clusters;
  pcl::IndicesPtr indices(new pcl::Indices);

  vector<arvc::plane> plane_clusters;

  arvc::viewer viewer("CLUSTERS");

  // MÉTODO 1
  clusters = arvc::euclideanClustering(input);

  cout << "Num of clusters: " << clusters.size() << endl;

// MÉTODO 2
  // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  // pcl::extractEuclideanClusters<pcl::PointXYZ>(*input, tree, 0.02, clusters, 100, 5000); //TODO : kdtree in projected plane, only in 2D

  for(pcl::PointIndices& cluster : clusters)
  {
    arvc::plane tmp_plane;
    pcl::PointIndicesPtr indices_ptr(new pcl::PointIndices);

    *indices_ptr = cluster;
    pcl::ModelCoefficientsPtr coeffs(new pcl::ModelCoefficients);
    coeffs->values = {0,0,1,0};

    tmp_plane.setPlane( coeffs, indices_ptr, input);

    viewer.addCloud(tmp_plane.cloud, tmp_plane.color);
    plane_clusters.push_back(tmp_plane);
  }

  // *indices = clusters[0].indices;
  // cloud_cluster = arvc::extract_indices(input, indices);
  // viewer.addCloud(cloud_cluster, arvc::color::RED_COLOR);
  viewer.show();

  return 0;
}
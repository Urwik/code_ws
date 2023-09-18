#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char** argv)
{
  // Check the number of command line arguments
  if (argc < 3)
  {
    std::cerr << "Usage: " << argv[0] << " input.ply output.pcd" << std::endl;
    return 1;
  }

  // Load the input PLY file
  pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZLNormal>);
  if (pcl::io::loadPLYFile(argv[1], *cloud) == -1)
  {
    std::cerr << "Failed to load input file " << argv[1] << std::endl;
    return 1;
  }

  // Create a new PCD file with x, y, z, and label fields
  pcl::PointCloud<pcl::PointXYZL>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZL>);
  output_cloud->width = cloud->width;
  output_cloud->height = cloud->height;
  output_cloud->is_dense = cloud->is_dense;
  output_cloud->points.resize(cloud->size());
  for (std::size_t i = 0; i < cloud->size(); ++i)
  {
    output_cloud->points[i].x = cloud->points[i].x;
    output_cloud->points[i].y = cloud->points[i].y;
    output_cloud->points[i].z = cloud->points[i].z;
    output_cloud->points[i].label = cloud->points[i].label;
  }

  // Save the output PCD file
  if (pcl::io::savePCDFileBinary(argv[2], *output_cloud) == -1)
  {
    std::cerr << "Failed to save output file " << argv[2] << std::endl;
    return 1;
  }

  return 0;
}
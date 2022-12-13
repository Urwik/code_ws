#include <iostream>
#include <fstream>
#include <filesystem>
#include <algorithm>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Type Definitions ////////////////////////////////////////////////////////////
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace fs = std::filesystem;

struct numPlanes
{
  std::string name;
  int value;
};


numPlanes getPlanesCount(fs::path path_to_cloud)
{
  // READ POINTCLOUD
  PointCloud::Ptr cloud (new PointCloud);

  std::string ext = path_to_cloud.extension();

  if(ext == ".pcd")
  {
    pcl::PCDReader reader;
    reader.read(path_to_cloud, *cloud);
  }  
  else
  {
    pcl::PLYReader reader;
    reader.read(path_to_cloud, *cloud);
  }
    
  std::vector<int> planes;
  for (auto point : *cloud)
  {
    planes.push_back(point.intensity);
  }

  int count;
  std::sort(planes.begin(), planes.end());
  count = std::unique(planes.begin(), planes.end()) - planes.begin();
  // count = std::distance(planes.begin(), std::unique(planes.begin(), planes.end()));

  std::cout << "For cloud " << path_to_cloud.stem() << ": " << count << std::endl;
  numPlanes num;
  num.name = path_to_cloud.stem();
  num.value = count - 1;
  return num;
}

void writePlanesCount(std::map<std::string, int> planesCount){
  std::ofstream myfile;
  myfile.open("planes_count.csv");

  for (auto const& [key, val] : planesCount)
  {
    myfile << key << "," << val << "\n";

  }
  myfile.close();
}

int main(int argc, char **argv)
{
  // Get handlres for source and target cloud data /////////////////////////////
  fs::path current_path = fs::current_path();

  if(argc < 2)
  {
    std::map<std::string, int> cloud_planes_count;
    std::vector<int> planes_count;
    for(const auto &entry : fs::directory_iterator(current_path))
    {
      numPlanes planes_count = getPlanesCount(entry.path());
      cloud_planes_count[planes_count.name] = planes_count.value;
    }
    writePlanesCount(cloud_planes_count);
  }
  else
  {
    std::string filename = argv[1];
    getPlanesCount(filename);
  }

  return 0;  
}



// C++
#include <iostream>
#include <stdlib.h>
#include <filesystem>


// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>


//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

typedef pcl::PointXYZLNormal PointT;
typedef pcl::PointCloud<PointT> PointCloud;
pcl::visualization::PCLVisualizer::Ptr pclVisualizer (new pcl::visualization::PCLVisualizer ("PCL Visualizer"));

// ************************************************************************** //
namespace fs = std::filesystem;
pcl::PCDReader pcd_reader;
pcl::PLYReader ply_reader;
////////////////////////////////////////////////////////////////////////////////

void plotCloud(fs::path path_)
{
  PointCloud::Ptr pc (new PointCloud);
  std::string file_ext = path_.extension();
  
  if (file_ext == ".pcd")
    pcd_reader.read(path_.string(), *pc);
  else if (file_ext == ".ply")
    ply_reader.read(path_.string(), *pc);
  else
    std::cout << "Format not compatible" << std::endl;

  

  

  std::stringstream ss;
  ss.str("");
  ss << path_.stem() << ".cam";
  std::string name = path_.stem();
  std::string new_name = name + ".cam"; 
  fs::path param_path = path_.parent_path() / new_name;
  std::cout << "Loading params from: " << param_path.string() << std::endl;

  pclVisualizer->initCameraParameters();
  pclVisualizer->loadCameraParameters(param_path);
  pclVisualizer->updateCamera();
  pclVisualizer->addPointCloud<PointT>(pc, "cloud");

  while (!pclVisualizer->wasStopped())
  {
    pclVisualizer->spinOnce(100);
  }
  
}



int main(int argc, char **argv)
{
  // Get handlres for source and target cloud data /////////////////////////////
  fs::path current_path = fs::current_path();

  if(argc < 2)
  {
    for(const auto &entry : fs::directory_iterator(current_path))
    {
      plotCloud(entry.path());
    }
  }
  else
  {
    fs::path input_file = current_path / argv[1];
    plotCloud(input_file);
  }

  return 0;
}

#include <iostream>
#include <filesystem>
#include <thread>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

// Type Definitions ////////////////////////////////////////////////////////////
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointXYZL PointI;
typedef pcl::PointCloud<PointI> PointCloudI;
typedef pcl::PointXYZL PointL;
typedef pcl::PointCloud<PointL> PointCloudL;

using namespace std;
namespace fs = std::filesystem;

  /**
   * @brief Lee una nube de puntos en formato .pcd o .ply
   * 
   * @param path Ruta de la nube de puntos
   * @return PointCloudI::Ptr 
   */
  PointCloudL::Ptr 
  readCloud (fs::path _path)
  {
    PointCloudL::Ptr _cloud_label (new PointCloudL);
    map<string, int> ext_map = {{".pcd", 0}, {".ply", 1}};

    switch (ext_map[_path.extension().string()])
    {
      case 0: {
        pcl::PCDReader pcd_reader;
        pcd_reader.read(_path.string(), *_cloud_label);
        break;
      }
      case 1: {
        pcl::PLYReader ply_reader;
        ply_reader.read(_path.string(), *_cloud_label);
        break;
      }
      default: {
        std::cout << "Format not compatible, it should be .pcd or .ply" << std::endl;
        break;
      }
    }

    return _cloud_label;
  }


void addCustomFrame(pcl::visualization::PCLVisualizer::Ptr &visualizer, const pcl::PointXYZ origin, const std::string name="origin", const float length = 0.5, const float radius = 0.1)
{

  visualizer->addSphere(origin, 0.15, 1.0, 0.0, 0.0, name + "_sphere");

  visualizer->addLine(origin, pcl::PointXYZ(origin.x + length, origin.y, origin.z), 1, 0, 0, name + "_x");
  visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, radius, name + "_x");

  visualizer->addLine(origin, pcl::PointXYZ(origin.x, origin.y + length, origin.z), 0, 1, 0, name + "_y");
  visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, radius, name + "_y");

  visualizer->addLine(origin, pcl::PointXYZ(origin.x, origin.y, origin.z + length), 0, 0, 1, name + "_z");
  visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, radius, name + "_z");
}


// void plotCloud(PointCloudL::Ptr &cloud)
// {

//   PointCloudL::Ptr truss_cloud (new PointCloudL);
//   PointCloudL::Ptr env_cloud (new PointCloudL);

//   for(auto &point : cloud->points)
//   {
//     if(point.label == 0)
//       env_cloud->push_back(point);
//     else
//       truss_cloud->push_back(point);
//   }


//   pcl::visualization::PCLVisualizer::Ptr visualizer (new pcl::visualization::PCLVisualizer("3D Viewer"));
//   visualizer->setBackgroundColor(1,1,1);

//   Eigen::Vector3f position(0,0,0);
//   pcl::PointXYZ origin(0,0,0);
//   visualizer->addSphere(origin, 0.2, 0.2, 0.2, 0.9, "origin_sphere");
//   pcl::visualization::PointCloudColorHandlerCustom<PointL> truss_color (truss_cloud, 50,190,50);
//   visualizer->addPointCloud<PointL>(truss_cloud, truss_color, "truss_cloud");

//   pcl::visualization::PointCloudColorHandlerCustom<PointL> env_color (env_cloud, 100, 100, 100);
//   visualizer->addPointCloud<PointL>(env_cloud, env_color, "env_cloud");

//   visualizer->addCoordinateSystem(0.7,0,0,0, "sensor_origin");


//   while (!visualizer->wasStopped())
//     visualizer->spinOnce(100);

// }


/**
 * @brief Converts a hue value (0-360 degrees) to RGB color
 * 
 * @param hue Hue angle in degrees (0-360)
 * @param max_hue Maximum hue value (default 360)
 * @return std::vector<int> RGB values [r, g, b] in range 0-255
 */
std::vector<int> hueColor(int hue, int max_hue=360)
{
  float r, g, b;
  float f = (float)hue / max_hue * 6.0;
  int i = floor(f);
  float q = f - i;
  float t = 1 - q;

  switch(i % 6) {
    case 0: r = 1; g = t; b = 0; break;
    case 1: r = q; g = 1; b = 0; break;
    case 2: r = 0; g = 1; b = t; break;
    case 3: r = 0; g = q; b = 1; break;
    case 4: r = t; g = 0; b = 1; break;
    case 5: r = 1; g = 0; b = q; break;
  }

  return {static_cast<int>(r * 255), static_cast<int>(g * 255), static_cast<int>(b * 255)};
}


int main(int argc, char **argv)
{
  fs::path current_path = fs::current_path();
  PointCloudL::Ptr cloud_in (new PointCloudL);
  pcl::PointXYZ origin(0,0,0);

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(1,1,1);

  try
  {
    viewer->loadCameraParameters("/home/arvc/tmp_cam_params.txt");
  }
  catch(const std::exception& e)
  {
  }


  if(argc < 2)
  {
    for(const auto &entry : fs::directory_iterator(current_path))
    {
      cloud_in = readCloud(entry);

      std::set<int> unique_labels;
      for (auto &point : cloud_in->points)
      {
        unique_labels.insert(point.label);
      }

      int max_label = *std::max_element(unique_labels.begin(), unique_labels.end());


      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      for (auto &point : cloud_in->points)
      {

        pcl::PointXYZRGB rgb_point;
        rgb_point.x = point.x;
        rgb_point.y = point.y;
        rgb_point.z = point.z;

        std::vector<int> color = hueColor(point.label * 360 / max_label);

        rgb_point.r = static_cast<uint8_t>(color[0]);
        rgb_point.g = static_cast<uint8_t>(color[1]);
        rgb_point.b = static_cast<uint8_t>(color[2]);
        
        colored_cloud->push_back(rgb_point);
      }

      viewer->removeAllPointClouds();
      viewer->removeAllShapes();

      viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "colored_cloud");

      addCustomFrame(viewer, origin, "sensor_origin", 1.5);

      viewer->spin();
      viewer->saveCameraParameters("/home/arvc/tmp_cam_params.txt");
    }
  }


  else
  {
    fs::path entry = argv[1];
    cloud_in = readCloud(entry);

    std::set<int> unique_labels;
    for (auto &point : cloud_in->points)
    {
      unique_labels.insert(point.label);
    }

    int max_label = *std::max_element(unique_labels.begin(), unique_labels.end());


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (auto &point : cloud_in->points)
    {

      pcl::PointXYZRGB rgb_point;
      rgb_point.x = point.x;
      rgb_point.y = point.y;
      rgb_point.z = point.z;

      if (point.label == 0)
      {
        rgb_point.r = 100;
        rgb_point.g = 100;
        rgb_point.b = 100;
      }
      else
      {

        std::vector<int> color = hueColor(point.label * 360 / max_label);

        rgb_point.r = static_cast<uint8_t>(color[0]);
        rgb_point.g = static_cast<uint8_t>(color[1]);
        rgb_point.b = static_cast<uint8_t>(color[2]);
      }
      
      colored_cloud->push_back(rgb_point);
    }

    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "colored_cloud");

    addCustomFrame(viewer, origin, "sensor_origin", 1.5);

    viewer->spin();
    viewer->saveCameraParameters("/home/arvc/tmp_cam_params.txt");
  }

  return 0;  
}
// C++
#include <iostream>
#include <filesystem>
#include <vector>
#include <algorithm>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Type Definitions ////////////////////////////////////////////////////////////
typedef pcl::PointXYZLNormal PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace fs = std::filesystem;

////////////////////////////////////////////////////////////////////////////////

PointCloud::Ptr readCloud(fs::path path_)
{
  PointCloud::Ptr cloud(new PointCloud);
  std::string file_ext = path_.extension();

  if (file_ext == ".pcd")
  {
    pcl::PCDReader pcd_reader;
    pcd_reader.read(path_.string(), *cloud);
  }
  else if (file_ext == ".ply")
  {
    pcl::PLYReader ply_reader;
    ply_reader.read(path_.string(), *cloud);
  }
  else
    std::cout << "Format not compatible, it should be .pcd or .ply" << std::endl;

  return cloud;
}

// Create color gradient from normalized curvature value (0.0 to 1.0)
void getColorFromGradient(float normalized_value, int &r, int &g, int &b)
{
  // Blue (low curvature) -> Cyan -> Green -> Yellow -> Red (high curvature)
  if (normalized_value < 0.25f) {
    // Blue to Cyan
    float local_val = normalized_value / 0.25f;
    r = 0;
    g = static_cast<int>(local_val * 255);
    b = 255;
  }
  else if (normalized_value < 0.5f) {
    // Cyan to Green
    float local_val = (normalized_value - 0.25f) / 0.25f;
    r = 0;
    g = 255;
    b = static_cast<int>((1.0f - local_val) * 255);
  }
  else if (normalized_value < 0.75f) {
    // Green to Yellow
    float local_val = (normalized_value - 0.5f) / 0.25f;
    r = static_cast<int>(local_val * 255);
    g = 255;
    b = 0;
  }
  else {
    // Yellow to Red
    float local_val = (normalized_value - 0.75f) / 0.25f;
    r = 255;
    g = static_cast<int>((1.0f - local_val) * 255);
    b = 0;
  }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr createCurvatureColorCloud(PointCloud::Ptr &cloud_in)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr gradient_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  gradient_cloud->resize(cloud_in->size());

  // Find min and max curvature values
  float min_curv = std::numeric_limits<float>::max();
  float max_curv = std::numeric_limits<float>::lowest();

  for (size_t i = 0; i < cloud_in->points.size(); i++)
  {
    float curv = cloud_in->points[i].curvature;
    if (curv < min_curv) min_curv = curv;
    if (curv > max_curv) max_curv = curv;
  }

  std::cout << "Curvature range: [" << min_curv << ", " << max_curv << "]" << std::endl;

  // Normalize curvature and apply color gradient
  float curv_range = max_curv - min_curv;
  if (curv_range < 1e-6f) curv_range = 1.0f; // Avoid division by zero

  for (size_t i = 0; i < cloud_in->points.size(); i++)
  {
    float normalized_curv = (cloud_in->points[i].curvature - min_curv) / curv_range;
    
    int r, g, b;
    getColorFromGradient(normalized_curv, r, g, b);

    gradient_cloud->points[i].x = cloud_in->points[i].x;
    gradient_cloud->points[i].y = cloud_in->points[i].y;
    gradient_cloud->points[i].z = cloud_in->points[i].z;
    gradient_cloud->points[i].r = r;
    gradient_cloud->points[i].g = g;
    gradient_cloud->points[i].b = b;
  }

  return gradient_cloud;
}

int main(int argc, char **argv)
{
  PointCloud::Ptr cloud_in(new PointCloud);
  fs::path current_dir = fs::current_path();

  if (argc < 2)
  {
    // Process all .pcd and .ply files in current directory
    std::vector<fs::path> path_vector;
    for (const auto &entry : fs::directory_iterator(current_dir))
    {
      if (entry.path().extension() == ".pcd" || entry.path().extension() == ".ply")
        path_vector.push_back(entry.path());
    }

    std::sort(path_vector.begin(), path_vector.end());

    if (path_vector.empty())
    {
      std::cout << "No .pcd or .ply files found in current directory." << std::endl;
      return 1;
    }

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Curvature Viewer"));
    viewer->setBackgroundColor(0.1, 0.1, 0.1);

    int index = 0;
    while (index >= 0 && index < (int)path_vector.size())
    {
      fs::path cloud_path = path_vector[index];
      std::cout << "\nLoading: " << cloud_path.filename() << " (" << (index + 1) << "/" << path_vector.size() << ")" << std::endl;
      
      cloud_in = readCloud(cloud_path);

      if (cloud_in->empty())
      {
        std::cout << "Warning: Cloud is empty, skipping..." << std::endl;
        index++;
        continue;
      }

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = createCurvatureColorCloud(cloud_in);

      viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "cloud");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
      
      std::string info_text = cloud_path.filename().string() + " (" + std::to_string(index + 1) + "/" + std::to_string(path_vector.size()) + ")";
      viewer->addText(info_text, 10, 10, 16, 1.0, 1.0, 1.0, "index_text");
      viewer->addText("Right: Next | Left: Previous | Q: Quit", 10, 30, 12, 0.8, 0.8, 0.8, "help_text");

      // Flag to control the spin loop
      bool should_continue_spinning = true;
      bool increase_index = false;

      // Register keyboard callback
      viewer->registerKeyboardCallback([&increase_index, &should_continue_spinning](const pcl::visualization::KeyboardEvent &event) {
        if (event.getKeySym() == "q" && event.keyDown())
        {
          should_continue_spinning = false;
          return;
        }
        if (event.getKeySym() == "Right" && event.keyDown())
        {
          should_continue_spinning = false;
          increase_index = true;
          return;
        }
        if (event.getKeySym() == "Left" && event.keyDown())
        {
          should_continue_spinning = false;
          increase_index = false;
          return;
        }
      });

      // Custom spin loop
      while (should_continue_spinning && !viewer->wasStopped())
      {
        viewer->spinOnce(100);
      }

      if (!should_continue_spinning)
      {
        if (increase_index)
          index++;
        else
          index = std::max(0, index - 1);

        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
      }

      if (viewer->wasStopped())
        break;
    }
  }
  else
  {
    // Process single file
    fs::path entry = argv[1];
    std::cout << "Loading: " << entry << std::endl;
    
    cloud_in = readCloud(entry);

    if (cloud_in->empty())
    {
      std::cout << "Error: Cloud is empty!" << std::endl;
      return 1;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = createCurvatureColorCloud(cloud_in);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Curvature Viewer"));
    viewer->setBackgroundColor(0.1, 0.1, 0.1);
    viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    viewer->addText(entry.filename().string(), 10, 10, 16, 1.0, 1.0, 1.0, "filename");

    while (!viewer->wasStopped())
      viewer->spinOnce(100);
  }

  return 0;
}



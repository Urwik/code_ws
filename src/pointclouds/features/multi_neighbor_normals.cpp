// C++
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

#include "tqdm.hpp"

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"  
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<pcl::Normal> Normals;
// ************************************************************************** //
namespace fs = std::filesystem;

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Lee una nube de puntos almacenada en un archivo.
 * 
 * @param path_ 
 * @return PointCloud::Ptr Devuelve la nube de puntos.
 */
PointCloud::Ptr 
readCloud(fs::path path_)
{
  PointCloud::Ptr cloud (new PointCloud);
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
  {
    std::cout << RED <<"Format not compatible, it should be .pcd or .ply" << RESET << std::endl;
    exit;
  }

  return cloud;
}


/**
 * @brief Calcula la normal de cada punto en función de la distribución de sus
 * X vecinos más cercanos
 * 
 * @param cloud_in Nube sobre la que se desean calcular las normales
 * @param neighbours Número de vecinos
 * @return pcl::PointCloud<pcl::Normal>::Ptr Normales asociadas a cada punto
 */
pcl::PointCloud<pcl::Normal>::Ptr 
computeNormals(PointCloud::Ptr &cloud_in, int neighbours = 30)
{
  // pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  
  ne.setInputCloud(cloud_in);
  ne.setSearchMethod(tree);
  // ne.setKSearch(neighbours);
  ne.setRadiusSearch(0.075); // Por radio existiran puntos cuya normal sea NaN //0.075
  ne.compute(*normals);

  // pcl::concatenateFields(*cloud_in, *normals, *cloud_out); 

  return normals;
}


/**
 * @brief Escribe la nube de puntos en formato .ply en un fichero
 * 
 * @param cloud_in Nube de entrada
 * @param entry Archivo de entrada
 */
void writeCloud(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_in, std::string name, std::string format)
{

  // Definicion ruta al archivo
  fs::path file_dir = fs::current_path();
  if (!fs::exists(file_dir)) 
    fs::create_directory(file_dir);

  std::string filename = name + '.' + format;
  fs::path abs_file_path = file_dir / filename;

  // Guardado del archivo en función de su formato
  if (format == "ply")
  {
    pcl::PLYWriter writer;
    writer.write(abs_file_path.string(), *cloud_in, true, false);
  }
  else if (format == "pcd")
  {
    pcl::PCDWriter writer;
    writer.write(abs_file_path.string(), *cloud_in, true);
  }
  else
  {
    std::cout << RED << "Invalid Format. Options: \"ply\" or \"pcd\"" << RESET << std::endl;
    exit;
  }
}


int main(int argc, char **argv)
{
  std::cout << GREEN << "STARTING CODE!!" << RESET << std::endl;
  auto start = std::chrono::high_resolution_clock::now();

  PointCloud::Ptr cloud_xyz (new PointCloud);
  Normals::Ptr normals (new Normals);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_xyznormals (new pcl::PointCloud<pcl::PointNormal>);

  fs::path entry = argv[1];
  cloud_xyz = readCloud(entry);


  std::vector<int> n_neighbors{74};
  std::stringstream ss;

  for (int neigh : n_neighbors)
  {
    normals = computeNormals(cloud_xyz, neigh);
    pcl::concatenateFields(*cloud_xyz, *normals, *cloud_xyznormals);
    ss.str("");
    ss << entry.stem().string() << '_' << neigh;
    writeCloud(cloud_xyznormals, ss.str(), "pcd");
  }
  

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "Computation Time: " << duration.count() << " ms" << std::endl;
  
  std::cout << GREEN << "COMPLETED!!" << RESET << std::endl;
  return 0;
}




// C++
#include <iostream>
#include <algorithm>
#include <filesystem>

// PCL
// #include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>

#include <Eigen/Core>
#include <eigen3/Eigen/Core>

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
// ************************************************************************** //
namespace fs = std::filesystem;
using namespace std;
////////////////////////////////////////////////////////////////////////////////


pcl::PCLPointCloud2::Ptr readCloud(fs::path path_)
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
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


void show_cloud_info(pcl::PCLPointCloud2::Ptr &cloud_in_)
{
  std::cout << "Width:  " << cloud_in_->width << std::endl;
  std::cout << "Height: " << cloud_in_->height << std::endl;
  std::cout << "Fields: ";
  for (size_t i = 0; i < cloud_in_->fields.size(); i++)
    std::cout << cloud_in_->fields[i] << ", ";
  std::cout << std::endl;

}


void pointcloud2_to_eigen(pcl::PCLPointCloud2::Ptr &cloud_in)
{
  int num_points = cloud_in->width * cloud_in->height;
  int num_fields = cloud_in->fields.size();
  
  Eigen::MatrixXf cloud_out = Eigen::MatrixXf::Ones(num_points, num_fields); 

  Eigen::ArrayXi offsets(num_fields);

  for (size_t i = 0; i < num_fields; i++)
    offsets[i] = cloud_in->fields[i].offset;

  std::cout << offsets << std::endl;
  std::cout << "point step " << cloud_in->point_step << std::endl;



  // for (size_t i = 0; i < num_points; i++)
  // {
  //   for (size_t j = 0; j < num_fields; j++)
  //   {
  //     memcpy(&cloud_out(i,j), &offsets[j], sizeof (float));
  //     // memcpy(&cloud_out(i,j), &offsets[j], sizeof (cloud_in->fields[j].datatype));

  //   }
  //   offsets += cloud_in->point_step;
    
  // }
  
  // std::cout << "Cloud Shape: " << std::endl;
  // std::cout << "[" << cloud_out.rows() << ", " << cloud_out.cols() << "]" << std::endl;

  // for (size_t i = 0; i < cloud_out.rows(); i++)
  // {
  //   for (size_t j = 0; j < cloud_out.cols(); j++)
  //   {
  //     std::cout << cloud_out(i,j) << ", ";
  //   }
  //   std::cout << std::endl;
  // }
  









  // if (in.fields[x_idx].datatype != pcl::PCLPointField::FLOAT32 ||
  //     in.fields[y_idx].datatype != pcl::PCLPointField::FLOAT32 ||
  //     in.fields[z_idx].datatype != pcl::PCLPointField::FLOAT32)
  // {
  //   PCL_ERROR ("X-Y-Z coordinates not floats. Currently only floats are supported.\n");
  //   return (false);
  // }

  // std::size_t npts = in.width * in.height;
  // out = Eigen::MatrixXf::Ones (4, npts);

  // Eigen::Array4i xyz_offset (in.fields[x_idx].offset, in.fields[y_idx].offset, in.fields[z_idx].offset, 0);

  // // Copy the input dataset into Eigen format
  // for (std::size_t i = 0; i < npts; ++i)
  // {
  //    // Unoptimized memcpys: assume fields x, y, z are in random order
  //    memcpy (&out (0, i), &in.data[xyz_offset[0]], sizeof (float));
  //    memcpy (&out (1, i), &in.data[xyz_offset[1]], sizeof (float));
  //    memcpy (&out (2, i), &in.data[xyz_offset[2]], sizeof (float));

  //    xyz_offset += in.point_step;
  // }

}


void xyz_pointcloud2_to_eigen(pcl::PCLPointCloud2::Ptr &cloud_in)
{

  vector<string> field_names;
  vector<size_t> field_index;

  for (size_t i = 0; i < cloud_in->fields.size(); i++)
  {
    if( string name = cloud_in->fields[i].name; name != "_")
    {
      field_names.push_back(name);
      field_index.push_back(i);
    }
  }

  int num_points = cloud_in->width * cloud_in->height;
  int num_fields = field_names.size();

  Eigen::MatrixXf cloud_out = Eigen::MatrixXf::Ones(num_points, num_fields); 
  cout << "OutCloud Shape: [" << cloud_out.rows() << ", " << cloud_out.cols() << "]" << endl;

  Eigen::ArrayXi offsets(num_fields);

  for (size_t i = 0; i < num_fields; i++)
    offsets[i] = cloud_in->fields[field_index[i]].offset;
  
  cout << "Offsets: " << offsets.transpose() << endl;

  for (size_t i = 0; i < field_names.size(); i++)
    cout << field_names[i] << ": " << (int) cloud_in->fields[field_index[i]].datatype << endl;


  for (size_t i = 0; i < num_points; i++)
  {
    for (size_t j = 0; j < num_fields; j++)
    {
      switch ((int) cloud_in->fields[j].datatype) {
        case 1:
          memcpy(&cloud_out(i,j), &cloud_in->data[offsets[j]], sizeof (int8_t));
          break;
        case 2:
          memcpy(&cloud_out(i,j), &cloud_in->data[offsets[j]], sizeof (uint8_t));
          break;
        case 3:
          memcpy(&cloud_out(i,j), &cloud_in->data[offsets[j]], sizeof (int16_t));
          break;
        case 4:
          memcpy(&cloud_out(i,j), &cloud_in->data[offsets[j]], sizeof (uint16_t));
          break;
        case 5:
          memcpy(&cloud_out(i,j), &cloud_in->data[offsets[j]], sizeof (int32_t));
          break;
        case 6:
          memcpy(&cloud_out(i,j), &cloud_in->data[offsets[j]], sizeof (uint32_t));
          break;
        case 7:
          memcpy(&cloud_out(i,j), &cloud_in->data[offsets[j]], sizeof (float));
          break;
        case 8:
          memcpy(&cloud_out(i,j), &cloud_in->data[offsets[j]], sizeof (double));
          break;
        default:
          cout << "Wrong data type for the field";
          break;
      }
    }
  
    offsets += cloud_in->point_step;
  }
  
  cout << cloud_out << endl;

}


int main(int argc, char **argv)
{
  pcl::console::setVerbosityLevel(pcl::console::VERBOSITY_LEVEL::L_ALWAYS);
  
  auto start = std::chrono::high_resolution_clock::now();

  pcl::PCLPointCloud2::Ptr cloud_in (new pcl::PCLPointCloud2);
  Eigen::MatrixXf cloud_out;



  fs::path current_dir = fs::current_path();
  if(argc < 2)
  {
    std::vector<fs::path> path_vector;
    for(const auto &entry : fs::directory_iterator(current_dir))
    {
      if(entry.path().extension() == ".pcd" || entry.path().extension() == ".ply")
        path_vector.push_back(entry.path());
    }

    for(const fs::path &entry : tq::tqdm(path_vector))
    {
      cloud_in = readCloud(entry);
      show_cloud_info(cloud_in);

    }
  }
  else
  {
    fs::path entry = argv[1];
    cloud_in = readCloud(entry);
    // pointcloud2_to_eigen(cloud_in);
    xyz_pointcloud2_to_eigen(cloud_in);


  }


  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "Computation Time: " << duration.count() << " ms" << std::endl;
  
  return 0;
}




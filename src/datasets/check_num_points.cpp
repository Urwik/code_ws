
#include "../pointclouds/arvc_utils.cpp"

namespace fs = std::filesystem;

//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"  
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"

typedef pcl::PointXYZLNormal PointT;
typedef pcl::PointCloud<PointT> PointCloud;

pcl::PCDReader pcd_reader;
pcl::PLYReader ply_reader;
fs::path current_path;
std::vector<int> n_points;
std::vector<std::string> cloud_names;
std::vector<std::string> error_files;
std::vector<std::string> readed_files;
std::vector<std::string> duplicated_files;


void check_num_points(PointCloud::Ptr &cloud_in, fs::path entry)
{
  if (cloud_in->points.size() <= 1024)
    std::cout << entry.filename() << ": " << cloud_in->points.size() << std::endl;  
}


////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  auto start = std::chrono::high_resolution_clock::now();
  PointCloud::Ptr cloud_in (new PointCloud);
  PointCloud::Ptr cloud_out (new PointCloud);

  fs::path current_dir = fs::current_path();


  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS); //OCULTA TODOS LOS MENSAJES DE PCL

  // EVERY CLOUD IN THE CURRENT FOLDER
  if(argc < 2)
  {
    std::vector<fs::path> path_vector;
    for(const auto &entry : fs::directory_iterator(current_dir))
    {
      if(entry.path().extension() == ".pcd" || entry.path().extension() == ".ply")
        path_vector.push_back(entry.path());
    }

    std::cout << "Clouds with less than 1024 points" << std::endl;
    for(const fs::path &entry : path_vector)
    {
      cloud_in = arvc::readCloud(entry);
      check_num_points(cloud_in, entry);
    }
    std::cout << std::endl; // Salto de linea despues de tqdm 

    // COMPUTATION TIME
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Computation Time: " << duration.count() << " ms" << std::endl;
  }
  // ONLY ONE CLOUD PASSED AS ARGUMENT IN CURRENT FOLDER
  else
  {
    fs::path entry = argv[1];
    cloud_in = arvc::readCloud(entry);
    check_num_points(cloud_in, entry);

    // COMPUTATION TIME
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Computation Time: " << duration.count() << " ms" << std::endl;
  }

  return 0;
}

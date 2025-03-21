#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <filesystem>
#include "tqdm.hpp"

namespace fs = std::filesystem;

std::string getCloudName(int num) {

    std::stringstream ss;
    ss << std::setfill('0') << std::setw(5)  << num;
    ss << ".pcd";

    return ss.str();
}

int main(int argc, char** argv)
{
    fs::path current_dir = fs::current_path();

    pcl::PointCloud<pcl::PointXYZL>::Ptr current (new pcl::PointCloud<pcl::PointXYZL>);
    pcl::PointCloud<pcl::PointXYZL>::Ptr next (new pcl::PointCloud<pcl::PointXYZL>);

    int current_size, next_size;

    pcl::PCDReader reader;
    pcl::VoxelGrid<pcl::PointXYZL> sor;

    std::vector<fs::path> cloud_paths;

    for (const auto & entry : fs::directory_iterator(current_dir))
    {
        if (entry.path().extension() != ".pcd")
            continue;
        else
            cloud_paths.push_back(entry.path());
    }

    int n_clouds = cloud_paths.size();

    for (int i=0 ; i< n_clouds; i++) {
        
        fs::path current_path = current_dir / getCloudName(i);
        fs::path next_path = current_dir / getCloudName(i+1);

        std::cout << "Checking clouds: " << current_path.stem() << " and " << next_path.stem() << ":  ";

        reader.read<pcl::PointXYZL>(current_path, *current);
        current_size = current->size();

        reader.read<pcl::PointXYZL>(next_path, *next);
        next_size = next->size();

        int diff =  std::abs(current_size - next_size);

        if (diff < current_size * 0.05) {
            std::cout << "\033[1;31mDuplicated\033[0m" << std::endl;
        }
        else {
            std::cout << "\033[1;32mNot duplicated\033[0m" << std::endl;
        }
    }
        
    return 0;
}



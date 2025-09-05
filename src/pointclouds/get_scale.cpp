#include <iostream>
#include <vector>
#include <algorithm>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include "read_cloud.hpp"

template<typename T>
float estimateCloudScale(const typename pcl::PointCloud<T>::Ptr &cloud) {
    if (cloud->points.empty()) {
        std::cerr << "Error: Point cloud is empty." << std::endl;
        return -1;
    }

    // Calculate the bounding box of the point cloud
    T min_pt, max_pt;
    pcl::getMinMax3D<T>(*cloud, min_pt, max_pt);

    // Calculate the scale based on the bounding box dimensions
    float depth = max_pt.z - min_pt.z;

    if (depth <= 0) {
        std::cerr << "Error: Invalid depth value." << std::endl;
        return -1;
    }

    return (depth > 100.0) ? 1000.0f : 1.0f;
}


int main(int argc, char** argv) {

    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <pointcloud_file>" << std::endl;
        std::cout << "Example: " << argv[0] << " cloud.pcd" << std::endl;
        return 1;
    }

    else if (argc == 2) {

        std::string first_arg = argv[1];

        if (first_arg == "-h" || first_arg == "--help") {
            std::cout << "Usage: " << argv[0] << " <pointcloud_file>" << std::endl;
            std::cout << "Example: " << argv[0] << " cloud.pcd" << std::endl;
            return 1;
        }
        else if (first_arg.substr(first_arg.find_last_of(".")) == ".pcd" ||
                 first_arg.substr(first_arg.find_last_of(".")) == ".ply") {

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            cloud = readPointCloud<pcl::PointXYZ>(argv[1]);

            if (!cloud || cloud->points.empty()) {
                std::cerr << "Error reading point cloud from file: " << argv[1] << std::endl;
                return 1;
            }
            float scale = estimateCloudScale<pcl::PointXYZ>(cloud);
            if (scale == 1.0) {
                std::cout << "Scale is in metres: " << scale << std::endl;
            }
            else if (scale == 1000.0) {
                std::cout << "Scale is in millimetres: " << scale << std::endl;
            }
            else {
                std::cout << "Scale could not be determined." << std::endl;
            }

            return 0;
        }
        else {
            std::cout << "Unknown option: " << argv[1] << std::endl;
            std::cout << "Use -h or --help for usage information." << std::endl;
            return 1;
        }
    }

    return 0;
}
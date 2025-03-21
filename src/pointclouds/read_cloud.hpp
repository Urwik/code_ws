#pragma once

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <filesystem>
#include <map>

namespace fs = std::filesystem;


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr readPointCloud(const std::string& path)
{
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    std::map<std::string, int> ext_map = { {".pcd", 0}, {".ply", 1} };

    switch (ext_map[fs::path(path).extension().string()])
    {
        case 0:
        {
            pcl::PCDReader reader;
            reader.read(path, *cloud);
            break;
        }
        case 1:
        {
            pcl::PLYReader reader;
            reader.read(path, *cloud);
            break;
        }
        default:
        {
            std::cout << "Format not compatible, it should be .pcd or .ply" << std::endl;
            break;
        }
    }

    return cloud;
}
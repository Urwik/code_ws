#pragma once
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/geometry.h>
#include <algorithm>
#include <limits>
#include <random>

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr farthestPointDownSample(typename pcl::PointCloud<PointT>::Ptr &cloud_in, const std::size_t target_points) {
    
    const std::size_t size = cloud_in->size();

    
    //if requested number of point is equal to the point cloud size, copy original cloud
    if (target_points == size) {
        std::cout << "Requested number of points is equal to the point cloud size!" << std::endl;
        return cloud_in;
    }

    //check if requested number of points is greater than the point cloud size
    if (target_points > size)
        std::cout << "Requested number of points is greater than point cloud size!" << std::endl;


    std::vector<float> distances_to_selected_points (size, std::numeric_limits<float>::max ());
    
    //set random seed
    std::mt19937 random_gen(74);
    std::uniform_int_distribution<std::size_t> dis(0, size -1);


    pcl::IndicesPtr indices (new pcl::Indices);
    //pick the first point at random
    std::size_t max_index = dis(random_gen);
    distances_to_selected_points[max_index] = -1.0;
    indices->push_back(max_index);
    
    for (std::size_t j = 1; j < target_points; ++j)
    {
        std::size_t next_max_index = 0;
        
        const PointT& max_index_point = (*cloud_in)[max_index];

        //recompute distances
        for (std::size_t i = 0; i < size; ++i)
        {
            if (distances_to_selected_points[i] == -1.0)
                continue;
        
            distances_to_selected_points[i] = std::min(distances_to_selected_points[i], pcl::geometry::distance((*cloud_in)[i], max_index_point));
        
            if (distances_to_selected_points[i] > distances_to_selected_points[next_max_index])
                next_max_index = i;
        }

        //select farthest point based on previously calculated distances
        //since distance is set to -1 for all selected elements,previously selected 
        //elements are guaranteed to not be selected
        max_index = next_max_index;
        distances_to_selected_points[max_index] = -1.0;
        indices->push_back(max_index);

        std::cout << "Size: " << indices->size() << std::endl;
        //set distance to -1 to ignore during max element search
    }

    typename pcl::PointCloud<PointT>::Ptr cloud_out(new pcl::PointCloud<PointT>);

    typename pcl::ExtractIndices<PointT> extract_indices;
    extract_indices.setInputCloud(cloud_in);
    extract_indices.setIndices(indices);
    extract_indices.setNegative(false);
    extract_indices.filter(*cloud_out);

    return cloud_out;


    // bool extract_removed_indices = true;
    // if (extract_removed_indices)
    // {
    //     for (std::size_t k = 0; k < distances_to_selected_points.size(); ++k)
    //     {
    //         if (distances_to_selected_points[k] != -1.0)
    //             (*removed_indices).push_back(k);
    //     }
    // }
}
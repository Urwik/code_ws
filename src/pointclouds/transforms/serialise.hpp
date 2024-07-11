#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>

#include <algorithm>
#include <vector>
#include <cmath>

template <typename PointT>
inline float calculateDistance(const PointT& point) {
    return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}

// Comparator function to sort points by distance
template <typename PointT>
bool comparePoints(const PointT& p1, const PointT& p2) {
    return calculateDistance(p1) < calculateDistance(p2);
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr sortCloudByDistance(typename pcl::PointCloud<PointT>::Ptr cloud) {

    // Create a copy of the input cloud
    typename pcl::PointCloud<PointT>::Ptr sortedCloud(new pcl::PointCloud<PointT>);

    *sortedCloud = *cloud;
    // Sort the points in the cloud by distance from the origin
    std::sort(sortedCloud->points.begin(), sortedCloud->points.end(), comparePoints<PointT>);

    return sortedCloud;
}
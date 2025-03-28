cmake_minimum_required(VERSION 3.16.3)
project(codeUtilities VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(THREADS_PREFER_PTHREAD_FLAG ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# Setting compile flags based on build type
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra")  # Debug information, no optimization
set(CMAKE_CXX_FLAGS_RELEASE "-O3")  # Optimized code for Release mode

# Apply the flags depending on build type
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${CMAKE_BUILD_TYPE}}")


# find_package(PCL 1.3 REQUIRED COMPONENTS common io filters visualization)
# find_package(catkin REQUIRED rosbag sensor_msgs std_msgs roscpp)
find_package(PCL 1.3 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(gazebo REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${Threads_INCLUDE_DIRS} )
link_directories(${catkin_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS} ${Eigen3_LIBRARY_DIRS} ${GAZEBO_LIBRARY_DIRS} ${Threads_LIBRARY_DIRS} )
add_definitions(${catkin_DEFINITIONS} ${PCL_DEFINITIONS} ${Eigen3_DEFINITIONS} ${GAZEBO_DEFINITIONS} ${Threads_DEFINITIONS} )

include_directories(${YAML_CPP_INCLUDE_DIRS})
link_directories(${YAML_CPP_INCLUDE_DIRS})
add_definitions(${YAML_CPP_DEFINITIONS})

# BAGFILES ....................................................................#
add_executable(arvc_remove_dot_from_extracted_bag_filenames src/bagfiles/remove_dot_from_extracted_bag_filenames.cpp)
target_link_libraries(arvc_remove_dot_from_extracted_bag_filenames ${catkin_LIBRARIES})

add_executable(arvc_bag_manage src/bagfiles/bag_manage.cpp)
target_link_libraries(arvc_bag_manage ${catkin_LIBRARIES})

# DATASETS ....................................................................#
add_executable(arvc_check_dataset src/datasets/check_dataset.cpp)
target_link_libraries(arvc_check_dataset ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(arvc_check_dataset_normals src/datasets/check_dataset_normals.cpp)
target_link_libraries(arvc_check_dataset_normals ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(arvc_rename_files src/datasets/rename_files.cpp)
target_link_libraries(arvc_rename_files)

add_executable(arvc_compute_plane_count src/datasets/compute_plane_count.cpp)
target_link_libraries(arvc_compute_plane_count ${PCL_LIBRARIES})

add_executable(arvc_compute_label_weights src/datasets/compute_label_weights.cpp)
target_link_libraries(arvc_compute_label_weights ${PCL_LIBRARIES})

add_executable(arvc_check_num_points src/datasets/check_num_points.cpp)
target_link_libraries(arvc_check_num_points ${PCL_LIBRARIES})

add_executable(arvc_check_real_dataset_pcd src/datasets/check_real_dataset_pcd.cpp)
target_link_libraries(arvc_check_real_dataset_pcd ${PCL_LIBRARIES})

add_executable(arvc_check_duplicated_clouds src/datasets/check_duplicated_clouds.cpp)
target_link_libraries(arvc_check_duplicated_clouds ${PCL_LIBRARIES})

# POINTCLOUDS .................................................................#
add_executable(arvc_pcd2ply src/pointclouds/transforms/pcd2ply.cpp)
target_link_libraries(arvc_pcd2ply ${PCL_LIBRARIES})

add_executable(arvc_compute_normals src/compute_normals.cpp)
target_link_libraries(arvc_compute_normals ${PCL_LIBRARIES})

# add_executable(arvc_normal_estimation src/pointclouds/features/compute_normals.cpp)
# target_link_libraries(arvc_compute_normals ${PCL_LIBRARIES})

# add_executable(arvc_compute_normals_XYZ src/pointclouds/compute_normals_XYZ.cpp)
# target_link_libraries(arvc_compute_normals_XYZ ${PCL_LIBRARIES})

# # add_executable(arvc_plane_segmentation_and_filtering src/pointclouds/plane_segmentation_and_filtering.cpp)
# # target_link_libraries(arvc_plane_segmentation_and_filtering ${PCL_LIBRARIES})

# add_executable(arvc_filter_ground_by_RANSAC src/pointclouds/filter_ground_by_RANSAC.cpp)
# target_link_libraries(arvc_filter_ground_by_RANSAC ${PCL_LIBRARIES})

# add_executable(arvc_view_cloud src/pointclouds/view_cloud.cpp)
# target_link_libraries(arvc_view_cloud ${PCL_LIBRARIES})

# add_executable(arvc_range_filter src/pointclouds/range_filter.cpp)
# target_link_libraries(arvc_range_filter ${PCL_LIBRARIES})

# add_executable(arvc_view_cloud_normals src/pointclouds/view_cloud_normals.cpp)
# target_link_libraries(arvc_view_cloud_normals ${PCL_LIBRARIES})

# add_executable(arvc_view_pcd_plane_by_plane src/pointclouds/view_pcd_plane_by_plane.cpp)
# target_link_libraries(arvc_view_pcd_plane_by_plane ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(arvc_plot_multiple_clouds src/pointclouds/plot_multiple_clouds.cpp)
# target_link_libraries(arvc_plot_multiple_clouds ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(arvc_plot_by_index src/pointclouds/plot_by_index.cpp)
# target_link_libraries(arvc_plot_by_index ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(arvc_random_sampling src/pointclouds/random_sampling.cpp)
# target_link_libraries(arvc_random_sampling ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(arvc_plot_by_binary_label src/pointclouds/visualization/plot_by_binary_label.cpp)
target_link_libraries(arvc_plot_by_binary_label ${PCL_LIBRARIES})

add_executable(arvc_plot_label_random_color src/pointclouds/visualization/plot_label_random_color.cpp)
target_link_libraries(arvc_plot_label_random_color ${PCL_LIBRARIES})

add_executable(arvc_plot_correct_vs_wrong src/pointclouds/visualization/plot_correct_vs_wrong.cpp)
target_link_libraries(arvc_plot_correct_vs_wrong ${PCL_LIBRARIES} ${Eigen3_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(arvc_plot_by_confusion_matrix src/pointclouds/visualization/plot_by_confusion_matrix.cpp)
target_link_libraries(arvc_plot_by_confusion_matrix ${PCL_LIBRARIES} ${Eigen3_LIBRARIES})

# add_executable(arvc_replace_field src/pointclouds/replace_field.cpp)
# target_link_libraries(arvc_replace_field ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(arvc_get_num_of_planes src/pointclouds/get_num_of_planes.cpp)
# target_link_libraries(arvc_get_num_of_planes ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(arvc_compute_roughness src/pointclouds/roughness.cpp)
# target_link_libraries(arvc_compute_roughness ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(arvc_pointcloud2_to_eigen src/pointclouds/pointcloud2_to_eigen.cpp)
# target_link_libraries(arvc_pointcloud2_to_eigen ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(arvc_multi_neighbor_normals src/pointclouds/multi_neighbor_normals.cpp)
# target_link_libraries(arvc_multi_neighbor_normals ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(arvc_plot_by_curvature src/pointclouds/plot_by_curvature.cpp)
# target_link_libraries(arvc_plot_by_curvature ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(arvc_clustering_regrow_normals src/pointclouds/clustering_regrow_normals.cpp)
# target_link_libraries(arvc_clustering_regrow_normals ${PCL_LIBRARIES})

add_executable(arvc_ground_segmentation_and_filtering src/pointclouds/segmentation/ground_segmentation_and_filtering.cpp)
target_link_libraries(arvc_ground_segmentation_and_filtering ${PCL_LIBRARIES})

add_executable(arvc_truss_plane_detector src/truss_plane_detector.cpp)
target_link_libraries(arvc_truss_plane_detector ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${GAZEBO_LIBRARIES} ${Eigen3_LIBRARIES})

add_executable(arvc_random_downsample src/random_downsample.cpp)
target_link_libraries(arvc_random_downsample ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

add_executable(arvc_main src/main.cpp)
target_link_libraries(arvc_main ${catkin_LIBRARIES} ${PCL_LIBRARIES} Threads::Threads)

add_executable(arvc_sort_cloud_by_distance src/sort_cloud_by_distance.cpp)
target_link_libraries(arvc_sort_cloud_by_distance ${PCL_LIBRARIES})
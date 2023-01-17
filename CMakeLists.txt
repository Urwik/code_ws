cmake_minimum_required(VERSION 3.16.3)
project(codeUtilities VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# find_package(PCL 1.3 REQUIRED COMPONENTS common io filters visualization)
find_package(catkin REQUIRED rosbag sensor_msgs std_msgs roscpp)
find_package(PCL 1.3 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)


include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS} ${Eigen3_LIBRARY_DIRS})
add_definitions(${catkin_DEFINITIONS} ${PCL_DEFINITIONS} ${Eigen3_DEFINITIONS})

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

# POINTCLOUDS .................................................................#
add_executable(arvc_pcd2ply src/pointclouds/pcd2ply.cpp)
target_link_libraries(arvc_pcd2ply ${PCL_LIBRARIES})

add_executable(arvc_compute_normals src/pointclouds/compute_normals.cpp)
target_link_libraries(arvc_compute_normals ${PCL_LIBRARIES})

add_executable(arvc_view_ply src/pointclouds/view_ply.cpp)
target_link_libraries(arvc_view_ply ${PCL_LIBRARIES})

add_executable(arvc_view_pcd src/pointclouds/view_pcd.cpp)
target_link_libraries(arvc_view_pcd ${PCL_LIBRARIES})

add_executable(arvc_view_cloud src/pointclouds/view_cloud.cpp)
target_link_libraries(arvc_view_cloud ${PCL_LIBRARIES})

add_executable(arvc_view_cloud_normals src/pointclouds/view_cloud_normals.cpp)
target_link_libraries(arvc_view_cloud_normals ${PCL_LIBRARIES})

add_executable(arvc_view_pcd_plane_by_plane src/pointclouds/view_pcd_plane_by_plane.cpp)
target_link_libraries(arvc_view_pcd_plane_by_plane ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(arvc_pcd_to_pcdNormals src/pointclouds/pcd_to_pcdNormals.cpp)
target_link_libraries(arvc_pcd_to_pcdNormals ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(arvc_plot_by_index src/pointclouds/plot_by_index.cpp)
target_link_libraries(arvc_plot_by_index ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(arvc_plot_by_binary_label src/pointclouds/plot_by_binary_label.cpp)
target_link_libraries(arvc_plot_by_binary_label ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(arvc_replace_field src/pointclouds/replace_field.cpp)
target_link_libraries(arvc_replace_field ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(arvc_get_num_of_planes src/pointclouds/get_num_of_planes.cpp)
target_link_libraries(arvc_get_num_of_planes ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(arvc_main src/main.cpp)
target_link_libraries(arvc_main ${catkin_LIBRARIES} ${PCL_LIBRARIES})
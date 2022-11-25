cmake_minimum_required(VERSION 3.16.3)
project(codeUtilities VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# find_package(PCL 1.3 REQUIRED COMPONENTS common io filters visualization)
find_package(PCL 1.3 REQUIRED)
find_package(catkin REQUIRED rosbag sensor_msgs std_msgs roscpp)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)


include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS} ${Eigen3_LIBRARY_DIRS})
add_definitions(${catkin_DEFINITIONS} ${PCL_DEFINITIONS} ${Eigen3_DEFINITIONS})


add_executable(arvc_pcd2ply src/pcd2ply.cpp)
target_link_libraries(arvc_pcd2ply ${PCL_LIBRARIES})

add_executable(arvc_view_ply src/view_ply.cpp)
target_link_libraries(arvc_view_ply ${PCL_LIBRARIES})

add_executable(arvc_view_pcd src/view_pcd.cpp)
target_link_libraries(arvc_view_pcd ${PCL_LIBRARIES})


add_executable(arvc_rename_files src/rename_files.cpp)
target_link_libraries(arvc_rename_files)

add_executable(arvc_change_bag_names src/change_bag_names.cpp)
target_link_libraries(arvc_change_bag_names ${catkin_LIBRARIES})

add_executable(arvc_bag_manage src/bag_manage.cpp)
target_link_libraries(arvc_bag_manage ${catkin_LIBRARIES})

add_executable(arvc_view_pcd_plane_by_plane src/view_pcd_plane_by_plane.cpp)
target_link_libraries(arvc_view_pcd_plane_by_plane ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(arvc_pcd_to_pcdNormals src/pcd_to_pcdNormals.cpp)
target_link_libraries(arvc_pcd_to_pcdNormals ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(arvc_plot_by_index src/plot_by_index.cpp)
target_link_libraries(arvc_plot_by_index ${catkin_LIBRARIES} ${PCL_LIBRARIES})

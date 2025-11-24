// C++
#include <iostream>
#include <stdlib.h>
#include <filesystem>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "tqdm.hpp"

//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

typedef pcl::PointXYZLNormal PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// ************************************************************************** //
namespace fs = std::filesystem;

////////////////////////////////////////////////////////////////////////////////

PointCloud::Ptr readCloud(fs::path path_)
{
	PointCloud::Ptr cloud(new PointCloud);
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

pcl::PointCloud<pcl::Normal>::Ptr getNormals(PointCloud::Ptr &cloud_in)
{
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	cloud_normals->points.resize(cloud_in->points.size());

	for (size_t i = 0; i < cloud_in->points.size(); i++)
	{
		cloud_normals->points[i].normal_x = cloud_in->points[i].normal_x;
		cloud_normals->points[i].normal_y = cloud_in->points[i].normal_y;
		cloud_normals->points[i].normal_z = cloud_in->points[i].normal_z;
		cloud_normals->points[i].curvature = cloud_in->points[i].curvature;
	}

	return cloud_normals;
}


int main(int argc, char **argv)
{
	pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZLNormal>);
	fs::path current_dir = fs::current_path();

	if (argc < 2)
	{
		std::vector<fs::path> path_vector;
		for (const auto &entry : fs::directory_iterator(current_dir))
		{
			if (entry.path().extension() == ".pcd" || entry.path().extension() == ".ply")
				path_vector.push_back(entry.path());
		}

		std::sort(path_vector.begin(), path_vector.end());

		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		// viewer->setBackgroundColor(1, 1, 1);

		int index = 0;
		while (index < (int) path_vector.size()) 
		{
			fs::path cloud_path = path_vector[index];
			cloud_in = readCloud(cloud_path);

			pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
			cloud_normals = getNormals(cloud_in);

			pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color(cloud_in, 0, 155, 0);
			viewer->addPointCloud<PointT>(cloud_in, cloud_color, "cloud");
			viewer->addPointCloudNormals<PointT, pcl::Normal>(cloud_in, cloud_normals, 10, 0.5, "normals");
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2.0, "normals");
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "normals");
			viewer->addText(std::to_string(index), 10, 10, 1.0, 1.0, 1.0, "index_text");

			// Flag to control the spin loop independently of the PCL visualizer's stopped state
			bool should_continue_spinning = true;
			
			bool increase_index = false;

			// Register keyboard callback to handle 'q' and right arrow keys
			viewer->registerKeyboardCallback([&increase_index, &should_continue_spinning](const pcl::visualization::KeyboardEvent& event) {
				if (event.getKeySym() == "q" && event.keyDown()) {
					// 'q' pressed: close window and kill viewer
					// viewer->close();
					should_continue_spinning = false;
					return;
				}
				if (event.getKeySym() == "Right" && event.keyDown()) {
					// Right arrow pressed: exit spin but keep viewer alive
					should_continue_spinning = false;
					increase_index = true;
					return;
				}
				if (event.getKeySym() == "Left" && event.keyDown()) {
					// Left arrow pressed: go back to previous cloud
					should_continue_spinning = false;
					increase_index = false;
					return;
				}
			});

			// Custom spin loop with interaction enabled
			while (should_continue_spinning && !viewer->wasStopped()) {
				viewer->spinOnce(100);
			}

			if (!should_continue_spinning) {

				if (increase_index)
					index++;
				else
					index = std::max(0, index - 1);


				viewer->removeAllPointClouds();
				viewer->removeAllShapes();
			}
		}
	}

	else
	{
		fs::path entry = argv[1];
		cloud_in = readCloud(entry);


		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
		cloud_normals = getNormals(cloud_in);

		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		viewer->setBackgroundColor(1, 1, 1);

		pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color(cloud_in, 0, 155, 0);
		viewer->addPointCloud<PointT>(cloud_in, cloud_color, "cloud");
		viewer->addPointCloudNormals<PointT, pcl::Normal>(cloud_in, cloud_normals, 10, 0.5, "normals");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "normals");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "normals");

		while (!viewer->wasStopped())
			viewer->spinOnce(100);
	}

	return 0;
}

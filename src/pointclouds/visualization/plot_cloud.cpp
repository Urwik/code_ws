#include <iostream>
#include <filesystem>
#include <thread>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

// Type Definitions ////////////////////////////////////////////////////////////
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointXYZL PointI;
typedef pcl::PointCloud<PointI> PointCloudI;
typedef pcl::PointXYZL PointL;
typedef pcl::PointCloud<PointL> PointCloudL;

using namespace std;
namespace fs = std::filesystem;

/**
 * @brief Lee una nube de puntos en formato .pcd o .ply
 *
 * @param path Ruta de la nube de puntos
 * @return PointCloudI::Ptr
 */
pcl::PCLPointCloud2Ptr
readCloud(fs::path _path)
{
	pcl::PCLPointCloud2Ptr cloud_blob(new pcl::PCLPointCloud2);
	map<string, int> ext_map = {{".pcd", 0}, {".ply", 1}};

	switch (ext_map[_path.extension().string()])
	{
		case 0:
		{
			pcl::PCDReader pcd_reader;
			pcd_reader.read(_path.string(), *cloud_blob);
			break;
		}
		case 1:
		{
			pcl::PLYReader ply_reader;
			ply_reader.read(_path.string(), *cloud_blob);
			break;
		}
		default:
		{
			std::cout << "Format not compatible, it should be .pcd or .ply" << std::endl;
			break;
		}
	}
	return cloud_blob;
}

bool hasField(const pcl::PCLPointCloud2& cloud, const std::string& name) {
    for (const auto& field : cloud.fields)
        if (field.name == name) return true;
    return false;
}

int main(int argc, char **argv)
{
	fs::path current_path = fs::current_path();
	pcl::PointXYZ origin(0, 0, 0);
	pcl::PCLPointCloud2Ptr cloud2(new pcl::PCLPointCloud2);

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(1, 1, 1);

	try
	{
		viewer->loadCameraParameters("/home/arvc/tmp_cam_params.txt");
	}
	catch (const std::exception &e)
	{
	}

	if (argc < 2)
	{
		for (const auto &entry : fs::directory_iterator(current_path))
		{
			cloud2 = readCloud(entry);

			viewer->removeAllPointClouds();
			viewer->removeAllShapes();


			bool hasRGB   = hasField(*cloud2, "rgb") || hasField(*cloud2, "rgba");
			bool hasNormals = hasField(*cloud2, "normal_x");
			bool hasIntensity = hasField(*cloud2, "intensity");
			bool hasLabels = hasField(*cloud2, "label") || hasField(*cloud2, "labels");

			if (hasRGB) {
				// XYZRGB cloud
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::fromPCLPointCloud2(*cloud2, *cloud);
				pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
				viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "cloud");
				std::cout << "Displaying XYZRGB cloud\n";
			} else if (hasNormals) {
				// XYZ + Normals
				pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
				pcl::fromPCLPointCloud2(*cloud2, *cloud);
				viewer->addPointCloud<pcl::PointNormal>(cloud, "cloud");
				viewer->addPointCloudNormals<pcl::PointNormal>(cloud, 10, 0.05f, "normals");
				std::cout << "Displaying PointNormal cloud\n";
			} else if (hasIntensity) {
				// XYZI cloud
				pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
				pcl::fromPCLPointCloud2(*cloud2, *cloud);

				std::set<uint32_t> unique_labels;
				std::unordered_map<uint32_t, pcl::PointCloud<pcl::PointXYZL>> label_to_cloud;

				for (size_t i = 0; i < cloud->points.size(); ++i) {
					uint32_t label = static_cast<uint32_t>(cloud->points[i].intensity);
					pcl::PointXYZL labeled_point;
					labeled_point.x = cloud->points[i].x;
					labeled_point.y = cloud->points[i].y;
					labeled_point.z = cloud->points[i].z;
					labeled_point.label = label;
					unique_labels.insert(label);
					label_to_cloud[label].points.push_back(labeled_point);
				}

				for (const auto& label : unique_labels) {
					int r = rand() % 256;
					int g = rand() % 256;
					int b = rand() % 256;
					pcl::PointCloud<pcl::PointXYZL>::Ptr label_cloud(new pcl::PointCloud<pcl::PointXYZL>);
					*label_cloud = label_to_cloud[label];
					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZL> label_color_handler(label_cloud, r, g, b);
					viewer->addPointCloud<pcl::PointXYZL>(label_cloud, label_color_handler, "cloud_label_" + std::to_string(label));
				}

				std::cout << "Displaying XYZI cloud\n";
			} else if (hasLabels) {
				// XYZL cloud
				pcl::PointCloud<pcl::PointXYZL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZL>);
				pcl::fromPCLPointCloud2(*cloud2, *cloud);

				std::set<uint32_t> unique_labels;
				std::unordered_map<uint32_t, pcl::PointCloud<pcl::PointXYZL>> label_to_cloud;

				for (size_t i = 0; i < cloud->points.size(); ++i) {
					uint32_t label = cloud->points[i].label;
					unique_labels.insert(label);
					label_to_cloud[label].points.push_back(cloud->points[i]);
				}

				for (const auto& label : unique_labels) {
					int r = rand() % 256;
					int g = rand() % 256;
					int b = rand() % 256;
					pcl::PointCloud<pcl::PointXYZL>::Ptr label_cloud(new pcl::PointCloud<pcl::PointXYZL>);
					*label_cloud = label_to_cloud[label];
					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZL> label_color_handler(label_cloud, r, g, b);
					viewer->addPointCloud<pcl::PointXYZL>(label_cloud, label_color_handler, "cloud_label_" + std::to_string(label));
				}
				std::cout << "Displaying XYZL cloud\n";
			} else {
				// Plain XYZ cloud
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::fromPCLPointCloud2(*cloud2, *cloud);
				viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
				std::cout << "Displaying XYZ cloud\n";
			} 

			// Flag to control the spin loop independently of the PCL visualizer's stopped state
			bool should_continue_spinning = true;

			// Register keyboard callback to handle 'q' and right arrow keys
			viewer->registerKeyboardCallback([&should_continue_spinning](const pcl::visualization::KeyboardEvent &event)
											 {
												if (event.getKeySym() == "q" && event.keyDown()) {
													// 'q' pressed: close window and kill viewer
													// viewer->
													should_continue_spinning = false;
													return;
												}
												if (event.getKeySym() == "Right" && event.keyDown()) {
													// Right arrow pressed: exit spin but keep viewer alive
													should_continue_spinning = false;
													return;
												} });

			// Custom spin loop with interaction enabled
			while (should_continue_spinning && !viewer->wasStopped())
			{
				viewer->spinOnce(100);
			}

			// If window was closed by user (X button) and not by 'q' key, clean up
			if (!should_continue_spinning && viewer->wasStopped())
			{
				viewer->removeAllPointClouds();
				viewer->removeAllShapes();
			}

			// viewer->spin();
			viewer->saveCameraParameters("/home/arvc/tmp_cam_params.txt");
		}
	}

	else
	{
		fs::path entry = argv[1];
		cloud2 = readCloud(entry);

		viewer->removeAllPointClouds();
		viewer->removeAllShapes();


		bool hasRGB   = hasField(*cloud2, "rgb") || hasField(*cloud2, "rgba");
		bool hasNormals = hasField(*cloud2, "normal_x");
		bool hasIntensity = hasField(*cloud2, "intensity");
		bool hasLabels = hasField(*cloud2, "label") || hasField(*cloud2, "labels");

		if (hasRGB) {
			// XYZRGB cloud
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::fromPCLPointCloud2(*cloud2, *cloud);
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
			viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "cloud");
			std::cout << "Displaying XYZRGB cloud\n";
		} else if (hasNormals) {
			// XYZ + Normals
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
			pcl::fromPCLPointCloud2(*cloud2, *cloud);
			viewer->addPointCloud<pcl::PointNormal>(cloud, "cloud");
			viewer->addPointCloudNormals<pcl::PointNormal>(cloud, 10, 0.05f, "normals");
			std::cout << "Displaying PointNormal cloud\n";
		} else if (hasIntensity) {
			// XYZI cloud
			pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
			pcl::fromPCLPointCloud2(*cloud2, *cloud);
			std::set<uint32_t> unique_labels;
			std::unordered_map<uint32_t, pcl::PointCloud<pcl::PointXYZL>> label_to_cloud;

			for (size_t i = 0; i < cloud->points.size(); ++i) {
				uint32_t label = static_cast<uint32_t>(cloud->points[i].intensity);
				pcl::PointXYZL labeled_point;
				labeled_point.x = cloud->points[i].x;
				labeled_point.y = cloud->points[i].y;
				labeled_point.z = cloud->points[i].z;
				labeled_point.label = label;
				unique_labels.insert(label);
				label_to_cloud[label].points.push_back(labeled_point);
			}

			for (const auto& label : unique_labels) {
				int r = rand() % 256;
				int g = rand() % 256;
				int b = rand() % 256;
				pcl::PointCloud<pcl::PointXYZL>::Ptr label_cloud(new pcl::PointCloud<pcl::PointXYZL>);
				*label_cloud = label_to_cloud[label];
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZL> label_color_handler(label_cloud, r, g, b);
				viewer->addPointCloud<pcl::PointXYZL>(label_cloud, label_color_handler, "cloud_label_" + std::to_string(label));
			}
			std::cout << "Displaying XYZI cloud\n";

		} else if (hasLabels) {
			// XYZL cloud
			pcl::PointCloud<pcl::PointXYZL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZL>);
			pcl::fromPCLPointCloud2(*cloud2, *cloud);

			std::set<uint32_t> unique_labels;
			std::unordered_map<uint32_t, pcl::PointCloud<pcl::PointXYZL>> label_to_cloud;

			for (size_t i = 0; i < cloud->points.size(); ++i) {
				uint32_t label = cloud->points[i].label;
				unique_labels.insert(label);
				label_to_cloud[label].points.push_back(cloud->points[i]);
			}

			for (const auto& label : unique_labels) {
				int r = rand() % 256;
				int g = rand() % 256;
				int b = rand() % 256;
				pcl::PointCloud<pcl::PointXYZL>::Ptr label_cloud(new pcl::PointCloud<pcl::PointXYZL>);
				*label_cloud = label_to_cloud[label];
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZL> label_color_handler(label_cloud, r, g, b);
				viewer->addPointCloud<pcl::PointXYZL>(label_cloud, label_color_handler, "cloud_label_" + std::to_string(label));
			}
			std::cout << "Displaying XYZL cloud\n";
		} else {
			// Plain XYZ cloud
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromPCLPointCloud2(*cloud2, *cloud);
			viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
			std::cout << "Displaying XYZ cloud\n";
		} 

		viewer->spin();
		viewer->saveCameraParameters("/home/arvc/tmp_cam_params.txt");
	}

	return 0;
}
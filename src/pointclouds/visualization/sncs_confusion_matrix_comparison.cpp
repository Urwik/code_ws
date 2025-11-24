#include <iostream>
#include <filesystem>
#include <thread>
#include <string>

#include "arvc_utils_v2.hpp"

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

using namespace std;
namespace fs = std::filesystem;

struct truss_idx
{
	pcl::Indices ground;
	pcl::Indices truss;
};

template <typename PointT>
truss_idx get_indices(typename pcl::PointCloud<PointT>::Ptr _cloud)
{
	truss_idx idx;

	for (size_t i = 0; i < _cloud->size(); i++)
	{
		if (_cloud->points[i].label == 0)
			idx.ground.push_back(i);
		else
			idx.truss.push_back(i);
	}

	return idx;
}

void addCustomFrame(pcl::visualization::PCLVisualizer::Ptr &visualizer, const pcl::PointXYZ origin, const std::string name="origin", const float length = 0.5, const float radius = 0.1, const int viewport=0)
{

  visualizer->addSphere(origin, 0.05, 1.0, 0.0, 0.0, name + "_sphere", viewport);

  visualizer->addLine(origin, pcl::PointXYZ(origin.x + length, origin.y, origin.z), 1, 0, 0, name + "_x", viewport);
  visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, radius, name + "_x", viewport);

  visualizer->addLine(origin, pcl::PointXYZ(origin.x, origin.y + length, origin.z), 0, 1, 0, name + "_y", viewport);
  visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, radius, name + "_y", viewport);

  visualizer->addLine(origin, pcl::PointXYZ(origin.x, origin.y, origin.z + length), 0, 0, 1, name + "_z", viewport);
  visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, radius, name + "_z", viewport);
}

struct ConfusionMatrixIndices
{
	pcl::IndicesPtr tp_idx;
	pcl::IndicesPtr tn_idx;
	pcl::IndicesPtr fp_idx;
	pcl::IndicesPtr fn_idx;

	ConfusionMatrixIndices() : tp_idx(new pcl::Indices), tn_idx(new pcl::Indices), fp_idx(new pcl::Indices), fn_idx(new pcl::Indices) {}

	int size() const
	{
		return tp_idx->size() + tn_idx->size() + fp_idx->size() + fn_idx->size();
	}
};

ConfusionMatrixIndices compute_cm_indices(const pcl::Indices &gt_truss, const pcl::Indices &gt_ground, const pcl::Indices pred_truss, const pcl::Indices &pred_ground)
{
	ConfusionMatrixIndices cm_indices;

	// True Positives (TP): Ground truth is truss and prediction is truss
	for (int idx : gt_truss)
	{
		if (std::find(pred_truss.begin(), pred_truss.end(), idx) != pred_truss.end())
		{
			cm_indices.tp_idx->push_back(idx);
		}
	}

	// True Negatives (TN): Ground truth is ground and prediction is ground
	for (int idx : gt_ground)
	{
		if (std::find(pred_ground.begin(), pred_ground.end(), idx) != pred_ground.end())
		{
			cm_indices.tn_idx->push_back(idx);
		}
	}

	// False Positives (FP): Ground truth is ground but prediction is truss
	for (int idx : gt_ground)
	{
		if (std::find(pred_truss.begin(), pred_truss.end(), idx) != pred_truss.end())
		{
			cm_indices.fp_idx->push_back(idx);
		}
	}

	// False Negatives (FN): Ground truth is truss but prediction is ground
	for (int idx : gt_truss)
	{
		if (std::find(pred_ground.begin(), pred_ground.end(), idx) != pred_ground.end())
		{
			cm_indices.fn_idx->push_back(idx);
		}
	}

	return cm_indices;
}

std::vector<std::string> split(const std::string &s, const std::string &delimiter)
{
	std::vector<std::string> tokens;
	size_t pos = 0;
	std::string token;
	std::string str = s;
	while ((pos = str.find(delimiter)) != std::string::npos)
	{
		token = str.substr(0, pos);
		tokens.push_back(token);
		str.erase(0, pos + delimiter.length());
	}
	tokens.push_back(str);

	return tokens;
}

void plot_by_confusion_matrix(const fs::path GT_PATH, const fs::path DL_PRED_PATH, const fs::path ANALYTICAL_PRED_PATH, const string CLOUD_NAME)
{

	// Create point cloud pointers
	PointCloud::Ptr cloud_in(new PointCloud);
	PointCloudL::Ptr gt_cloud(new PointCloudL);
	PointCloudL::Ptr dl_pred_cloud(new PointCloudL);
	PointCloudL::Ptr adhoc_pred_cloud(new PointCloudL);

	// Load point clouds
	fs::path gt_cloud_path = GT_PATH / (CLOUD_NAME + ".ply");
	fs::path dl_pred_path = DL_PRED_PATH / (CLOUD_NAME + ".ply");
	fs::path adhoc_pred_path = ANALYTICAL_PRED_PATH / (CLOUD_NAME + ".ply");

	gt_cloud = arvc::readPointCloud<PointL>(gt_cloud_path);
	dl_pred_cloud = arvc::readPointCloud<PointL>(dl_pred_path);
	adhoc_pred_cloud = arvc::readPointCloud<PointL>(adhoc_pred_path);
	cloud_in = arvc::parseToXYZ(gt_cloud);

	if (gt_cloud->points.size() != dl_pred_cloud->points.size())
	{
		std::cout << "Error: GT and Pred clouds have different sizes" << std::endl;
		std::cout << "GT size: " << gt_cloud->points.size() << ", Pred size: " << dl_pred_cloud->points.size() << std::endl;
		return;
	}

	std::cout << "GT Cloud Path: " << gt_cloud_path.string() << std::endl;
	std::cout << "DLPred Cloud Path: " << dl_pred_path.string() << std::endl;
	std::cout << "Adhoc Pred Cloud Path: " << adhoc_pred_path.string() << std::endl;

	// Get Gt and Pred Indices
	truss_idx gt;
	truss_idx dl_pred;
	truss_idx adhoc_pred;
	gt = get_indices<PointL>(gt_cloud);
	dl_pred = get_indices<PointL>(dl_pred_cloud);
	adhoc_pred = get_indices<PointL>(adhoc_pred_cloud);

	// Compute Confusion Matrix Indices for dl method
	ConfusionMatrixIndices dl_cm_indices;
	dl_cm_indices = compute_cm_indices(gt.truss, gt.ground, dl_pred.truss, dl_pred.ground);
	if (dl_cm_indices.size() != (int)cloud_in->size())
	{
		std::cout << "Error: Confusion Matrix size is not equal to the cloud size" << std::endl;
		std::cout << "CM size: " << dl_cm_indices.size() << ", Cloud size: " << cloud_in->size() << std::endl;
		return;
	}

	PointCloud::Ptr dl_tp_cloud(new PointCloud);
	PointCloud::Ptr dl_tn_cloud(new PointCloud);
	PointCloud::Ptr dl_fp_cloud(new PointCloud);
	PointCloud::Ptr dl_fn_cloud(new PointCloud);
	pcl::IndicesPtr dl_error_idx(new pcl::Indices);

	dl_error_idx->insert(dl_error_idx->end(), dl_cm_indices.fp_idx->begin(), dl_cm_indices.fp_idx->end());
	dl_error_idx->insert(dl_error_idx->end(), dl_cm_indices.fn_idx->begin(), dl_cm_indices.fn_idx->end());

	dl_tp_cloud = arvc::extract_indices(cloud_in, dl_cm_indices.tp_idx, false);
	dl_tn_cloud = arvc::extract_indices(cloud_in, dl_cm_indices.tn_idx, false);
	dl_fp_cloud = arvc::extract_indices(cloud_in, dl_cm_indices.fp_idx, false);
	dl_fn_cloud = arvc::extract_indices(cloud_in, dl_cm_indices.fn_idx, false);

	std::cout << CLOUD_NAME << " -- Miou: " << (float)dl_cm_indices.tp_idx->size() / (dl_cm_indices.tp_idx->size() + dl_cm_indices.fp_idx->size() + dl_cm_indices.fn_idx->size()) << std::endl;
	std::cout << CLOUD_NAME << " -- Recall: " << (float)dl_cm_indices.tp_idx->size() / (dl_cm_indices.tp_idx->size() + dl_cm_indices.fn_idx->size()) << std::endl;


	// Compute Confusion Matrix Indices for adhoc method
	ConfusionMatrixIndices adhoc_cm_indices;
	adhoc_cm_indices = compute_cm_indices(gt.truss, gt.ground, adhoc_pred.truss, adhoc_pred.ground);
	if (adhoc_cm_indices.size() != (int)cloud_in->size())
	{
		std::cout << "Error: Confusion Matrix size is not equal to the cloud size" << std::endl;
		std::cout << "CM size: " << adhoc_cm_indices.size() << ", Cloud size: " << cloud_in->size() << std::endl;
		return;
	}

	PointCloud::Ptr adhoc_tp_cloud(new PointCloud);
	PointCloud::Ptr adhoc_tn_cloud(new PointCloud);
	PointCloud::Ptr adhoc_fp_cloud(new PointCloud);
	PointCloud::Ptr adhoc_fn_cloud(new PointCloud);
	pcl::IndicesPtr adhoc_error_idx(new pcl::Indices);

	adhoc_error_idx->insert(adhoc_error_idx->end(), adhoc_cm_indices.fp_idx->begin(), adhoc_cm_indices.fp_idx->end());
	adhoc_error_idx->insert(adhoc_error_idx->end(), adhoc_cm_indices.fn_idx->begin(), adhoc_cm_indices.fn_idx->end());

	adhoc_tp_cloud = arvc::extract_indices(cloud_in, adhoc_cm_indices.tp_idx, false);
	adhoc_tn_cloud = arvc::extract_indices(cloud_in, adhoc_cm_indices.tn_idx, false);
	adhoc_fp_cloud = arvc::extract_indices(cloud_in, adhoc_cm_indices.fp_idx, false);
	adhoc_fn_cloud = arvc::extract_indices(cloud_in, adhoc_cm_indices.fn_idx, false);

	std::cout << CLOUD_NAME << " -- Miou: " << (float)adhoc_cm_indices.tp_idx->size() / (adhoc_cm_indices.tp_idx->size() + adhoc_cm_indices.fp_idx->size() + adhoc_cm_indices.fn_idx->size()) << std::endl;
	std::cout << CLOUD_NAME << " -- Recall: " << (float)adhoc_cm_indices.tp_idx->size() / (adhoc_cm_indices.tp_idx->size() + adhoc_cm_indices.fn_idx->size()) << std::endl;



	// -------------------------------- //
	// Visualization Stuff
	// -------------------------------- //
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer(CLOUD_NAME));
	fs::path cam_params_dir = GT_PATH.parent_path();
	fs::path camera_params_path = cam_params_dir / (CLOUD_NAME + "_cam_params.txt");

	int v1(0);
	int v2(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1); // Left viewport
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2); // Right viewport
	viewer->setBackgroundColor(1, 1, 1, v1);
	viewer->setBackgroundColor(1, 1, 1, v2);

	viewer->addText("DL Method", 10, 10, "v1_text", v1);
	viewer->addText("Analytical Method", 10, 10, "v2_text", v2);

	try
	{
		viewer->loadCameraParameters(camera_params_path.string());
	}
	catch (const std::exception &e)
	{
		std::cout << "No camera parameters found" << std::endl;
	}

	addCustomFrame(viewer, pcl::PointXYZ(.0, .0, .0), "origin1", 1.0, 2.0, v1);
	addCustomFrame(viewer, pcl::PointXYZ(.0, .0, .0), "origin2", 1.0, 2.0, v2);
	// pcl::visualization::PointCloudColorHandlerCustom<PointT> tp_color (tp_cloud, 76, 175, 80); //76, 175, 80); original
	// pcl::visualization::PointCloudColorHandlerCustom<PointT> tn_color (tn_cloud, 100,100,100); //100,100,100); original
	// pcl::visualization::PointCloudColorHandlerCustom<PointT> fp_color (fp_cloud, 211, 47, 47); //211, 47, 47); original
	// pcl::visualization::PointCloudColorHandlerCustom<PointT> fn_color (fn_cloud, 255, 152, 0); //255, 152, 0); original

	pcl::visualization::PointCloudColorHandlerCustom<PointT> dl_tp_color(dl_tp_cloud, 50, 190, 50);	// sncs_color_palette
	pcl::visualization::PointCloudColorHandlerCustom<PointT> dl_tn_color(dl_tn_cloud, 100, 100, 100); // sncs_color_palette
	pcl::visualization::PointCloudColorHandlerCustom<PointT> dl_fp_color(dl_fp_cloud, 200, 10, 10);	// sncs_color_palette
	pcl::visualization::PointCloudColorHandlerCustom<PointT> dl_fn_color(dl_fn_cloud, 200, 10, 10);	// sncs_color_palette


	pcl::visualization::PointCloudColorHandlerCustom<PointT> adhoc_tp_color(adhoc_tp_cloud, 50, 190, 50);	// sncs_color_palette
	pcl::visualization::PointCloudColorHandlerCustom<PointT> adhoc_tn_color(adhoc_tn_cloud, 100, 100, 100); // sncs_color_palette
	pcl::visualization::PointCloudColorHandlerCustom<PointT> adhoc_fp_color(adhoc_fp_cloud, 200, 10, 10);	// sncs_color_palette
	pcl::visualization::PointCloudColorHandlerCustom<PointT> adhoc_fn_color(adhoc_fn_cloud, 200, 10, 10);	// sncs_color_palette

	viewer->addPointCloud(dl_tp_cloud, dl_tp_color, "dl_tp_cloud", v1);
	viewer->addPointCloud(dl_tn_cloud, dl_tn_color, "dl_tn_cloud", v1);
	viewer->addPointCloud(dl_fp_cloud, dl_fp_color, "dl_fp_cloud", v1);
	viewer->addPointCloud(dl_fn_cloud, dl_fn_color, "dl_fn_cloud", v1);

	viewer->addPointCloud(adhoc_tp_cloud, adhoc_tp_color, "adhoc_tp_cloud", v2);
	viewer->addPointCloud(adhoc_tn_cloud, adhoc_tn_color, "adhoc_tn_cloud", v2);
	viewer->addPointCloud(adhoc_fp_cloud, adhoc_fp_color, "adhoc_fp_cloud", v2);
	viewer->addPointCloud(adhoc_fn_cloud, adhoc_fn_color, "adhoc_fn_cloud", v2);


	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "dl_tp_cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "dl_tn_cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "dl_fp_cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "dl_fn_cloud");

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "adhoc_tp_cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "adhoc_tn_cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "adhoc_fp_cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "adhoc_fn_cloud");

	while (!viewer->wasStopped())
	{
		viewer->saveCameraParameters(camera_params_path.string());
		viewer->spinOnce(1000);
	}
}

int main()
{
	const std::string SET_ID = "05";

	fs::path GT_PATH("/media/arvc/data/datasets/sncs_test/v1/" + SET_ID + "/ply_xyzln");
	fs::path DL_PRED_PATH("/media/arvc/data/sncs_revision/sncs_dl_results/PointNet2BinSeg/240723142940/sncs_test_unfixed/v1/" + SET_ID); // DL METHODS
	fs::path ANALYTICAL_PRED_PATH("/home/arvc/workspaces/arvc_ws/src/arvc_ground_filter/results_sncs_revision_single_threaded/" + SET_ID + "/2"); // ANALYTICAL

	// GET POINTCLOUDS FILES
	std::vector<fs::path> cloud_paths;
	for (const auto &entry : fs::directory_iterator(GT_PATH))
	{
		if (entry.path().extension() == ".ply" || entry.path().extension() == ".pcd")
			cloud_paths.push_back(entry.path());
	}
	std::sort(cloud_paths.begin(), cloud_paths.end());

	// ITERATE OVER FILES
	for (const auto &entry : cloud_paths)
	{
		std::string CLOUD_NAME = entry.stem().string();
		plot_by_confusion_matrix(GT_PATH, DL_PRED_PATH, ANALYTICAL_PRED_PATH, CLOUD_NAME);
	}
	return 0;
}

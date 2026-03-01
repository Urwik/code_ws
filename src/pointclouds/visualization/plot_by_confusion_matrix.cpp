#include <iostream>
#include <filesystem>
#include <thread>
#include <string>
#include <unordered_set>

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

void addCustomFrame(pcl::visualization::PCLVisualizer::Ptr &visualizer, const pcl::PointXYZ origin, const std::string name="origin", const float length = 0.5, const float radius = 0.1)
{

  visualizer->addSphere(origin, 0.05, 1.0, 0.0, 0.0, name + "_sphere");

  visualizer->addLine(origin, pcl::PointXYZ(origin.x + length, origin.y, origin.z), 1, 0, 0, name + "_x");
  visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, radius, name + "_x");

  visualizer->addLine(origin, pcl::PointXYZ(origin.x, origin.y + length, origin.z), 0, 1, 0, name + "_y");
  visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, radius, name + "_y");

  visualizer->addLine(origin, pcl::PointXYZ(origin.x, origin.y, origin.z + length), 0, 0, 1, name + "_z");
  visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, radius, name + "_z");
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

ConfusionMatrixIndices 
compute_cm_indices(const pcl::Indices &gt_truss, const pcl::Indices &gt_ground, const pcl::Indices &pred_truss, const pcl::Indices &pred_ground)
{
    ConfusionMatrixIndices cm_indices;

    // Use hash sets for O(1) average time complexity lookups
    std::unordered_set<int> pred_truss_set(pred_truss.begin(), pred_truss.end());
    std::unordered_set<int> pred_ground_set(pred_ground.begin(), pred_ground.end());

    // True Positives (TP): Ground truth is truss and prediction is truss
    for (int idx : gt_truss)
    {
        if (pred_truss_set.count(idx))
        {
            cm_indices.tp_idx->push_back(idx);
        }
    }

    // True Negatives (TN): Ground truth is ground and prediction is ground
    for (int idx : gt_ground)
    {
        if (pred_ground_set.count(idx))
        {
            cm_indices.tn_idx->push_back(idx);
        }
    }

    // False Positives (FP): Ground truth is ground but prediction is truss
    for (int idx : gt_ground)
    {
        if (pred_truss_set.count(idx))
        {
            cm_indices.fp_idx->push_back(idx);
        }
    }

    // False Negatives (FN): Ground truth is truss but prediction is ground
    for (int idx : gt_truss)
    {
        if (pred_ground_set.count(idx))
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

void plot_by_confusion_matrix(const fs::path GT_PATH, const fs::path PRED_PATH, const string CLOUD_NAME)
{

	// Create point cloud pointers
	PointCloud::Ptr cloud_in(new PointCloud);
	PointCloudL::Ptr gt_cloud(new PointCloudL);
	PointCloudL::Ptr pred_cloud(new PointCloudL);

	// Load point clouds
	fs::path gt_cloud_path = GT_PATH / (CLOUD_NAME + ".pcd");
	fs::path pred_cloud_path = PRED_PATH / (CLOUD_NAME + ".ply");

	gt_cloud = arvc::readPointCloud<PointL>(gt_cloud_path);
	pred_cloud = arvc::readPointCloud<PointL>(pred_cloud_path);
	cloud_in = arvc::parseToXYZ(gt_cloud);

	if (gt_cloud->points.size() != pred_cloud->points.size())
	{
		std::cout << "Error: GT and Pred clouds have different sizes" << std::endl;
		std::cout << "GT size: " << gt_cloud->points.size() << ", Pred size: " << pred_cloud->points.size() << std::endl;
		return;
	}

	std::cout << "GT Cloud Path: " << gt_cloud_path.string() << std::endl;
	std::cout << "Pred Cloud Path: " << pred_cloud_path.string() << std::endl;

	// Get Gt and Pred Indices
	truss_idx gt;
	truss_idx pred;
	gt = get_indices<PointL>(gt_cloud);
	pred = get_indices<PointL>(pred_cloud);

	// Compute Confusion Matrix Indices
	ConfusionMatrixIndices cm_indices;
	cm_indices = compute_cm_indices(gt.truss, gt.ground, pred.truss, pred.ground);

	if (cm_indices.size() != (int)cloud_in->size())
	{
		std::cout << "Error: Confusion Matrix size is not equal to the cloud size" << std::endl;
		std::cout << "CM size: " << cm_indices.size() << ", Cloud size: " << cloud_in->size() << std::endl;
		return;
	}

	PointCloud::Ptr tp_cloud(new PointCloud);
	PointCloud::Ptr tn_cloud(new PointCloud);
	PointCloud::Ptr fp_cloud(new PointCloud);
	PointCloud::Ptr fn_cloud(new PointCloud);
	pcl::IndicesPtr error_idx(new pcl::Indices);

	error_idx->insert(error_idx->end(), cm_indices.fp_idx->begin(), cm_indices.fp_idx->end());
	error_idx->insert(error_idx->end(), cm_indices.fn_idx->begin(), cm_indices.fn_idx->end());

	tp_cloud = arvc::extract_indices(cloud_in, cm_indices.tp_idx, false);
	tn_cloud = arvc::extract_indices(cloud_in, cm_indices.tn_idx, false);
	fp_cloud = arvc::extract_indices(cloud_in, cm_indices.fp_idx, false);
	fn_cloud = arvc::extract_indices(cloud_in, cm_indices.fn_idx, false);

	std::cout << CLOUD_NAME << " -- Miou: " << (float)cm_indices.tp_idx->size() / (cm_indices.tp_idx->size() + cm_indices.fp_idx->size() + cm_indices.fn_idx->size()) << std::endl;
	std::cout << CLOUD_NAME << " -- Recall: " << (float)cm_indices.tp_idx->size() / (cm_indices.tp_idx->size() + cm_indices.fn_idx->size()) << std::endl;

	// -------------------------------- //
	// Visualization Stuff
	// -------------------------------- //
	pcl::visualization::PCLVisualizer::Ptr my_vis (new pcl::visualization::PCLVisualizer(pred_cloud_path.stem().string()));
	fs::path cam_params_dir = GT_PATH.parent_path();
	fs::path camera_params_path = cam_params_dir / (CLOUD_NAME + "_cam_params.txt");

	try
	{
		my_vis->loadCameraParameters(camera_params_path.string());
	}
	catch (const std::exception &e)
	{
		std::cout << "No camera parameters found" << std::endl;
	}

	my_vis->setBackgroundColor(1, 1, 1);

	addCustomFrame(my_vis, pcl::PointXYZ(.0, .0, .0), "origin", 1.0, 2.0);
	// pcl::visualization::PointCloudColorHandlerCustom<PointT> tp_color (tp_cloud, 76, 175, 80); //76, 175, 80); original
	// pcl::visualization::PointCloudColorHandlerCustom<PointT> tn_color (tn_cloud, 100,100,100); //100,100,100); original
	// pcl::visualization::PointCloudColorHandlerCustom<PointT> fp_color (fp_cloud, 211, 47, 47); //211, 47, 47); original
	// pcl::visualization::PointCloudColorHandlerCustom<PointT> fn_color (fn_cloud, 255, 152, 0); //255, 152, 0); original

	pcl::visualization::PointCloudColorHandlerCustom<PointT> tp_color(tp_cloud, 50, 190, 50);	// sncs_color_palette
	pcl::visualization::PointCloudColorHandlerCustom<PointT> tn_color(tn_cloud, 100, 100, 100); // sncs_color_palette
	pcl::visualization::PointCloudColorHandlerCustom<PointT> fp_color(fp_cloud, 200, 10, 10);	// sncs_color_palette
	pcl::visualization::PointCloudColorHandlerCustom<PointT> fn_color(fn_cloud, 200, 10, 10);	// sncs_color_palette

	my_vis->addPointCloud(tp_cloud, tp_color, "tp_cloud");
	my_vis->addPointCloud(tn_cloud, tn_color, "tn_cloud");
	my_vis->addPointCloud(fp_cloud, fp_color, "fp_cloud");
	my_vis->addPointCloud(fn_cloud, fn_color, "fn_cloud");

	my_vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "tp_cloud");
	my_vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "tn_cloud");
	my_vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "fp_cloud");
	my_vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "fn_cloud");

	while (!my_vis->wasStopped())
	{
		my_vis->saveCameraParameters(camera_params_path.string());
		my_vis->spinOnce(1000);
	}

	my_vis->saveScreenshot(PRED_PATH.string() + "/" + (CLOUD_NAME + "_conf_matrix.png"));
}

int main(int argc, char** argv)
{
	// std::vector<std::string> MODELS = {"PointNet2BinSeg", "MinkUNet34C", "PointTransformerV3"};
	// std::vector<std::string> FEATURES = {"c","nxnynz", "xyz", "xyzc", "xyznxnynz"};
	// std::vector<std::string> DATASETS = {"00", "01", "02"};

	// std::vector<std::string> MODEL_RESULTS = {
	//   "/home/fran/Documents/2_ARTICULOS/RIAI25_complex_truss_analysis/PointNet2BinSeg/240722104532/complex_structure_v2",
	//   "/home/fran/Documents/2_ARTICULOS/RIAI25_complex_truss_analysis/MinkUNet34C/240628235025/complex_structure_v2",
	//   "/home/fran/Documents/2_ARTICULOS/RIAI25_complex_truss_analysis/PointTransformerV3/c/complex_structure_v2"
	// };

	// // std::string MODEL_NAME = MODELS[3];
	// // std::string FEATURE = FEATURES[0];
	// std::string SET_NAME = DATASETS[0];
	// std::string SUFFIX = "ply_xyzln_fixedSize";

	// // MULTIPLE MODELS AN FILES
	// for (const std::string& model : MODEL_RESULTS)
	// {
	//   for (const std::string& dataset : DATASETS)
	//   {
	//     std::vector<std::string> parts = split(model, "/");
	//     const std::string MODEL_NAME = parts[parts.size() - 3]; // Get the third-to-last part of the path as model name
	//     SET_NAME = dataset;

	//     std::cout << "Processing model: " << MODEL_NAME << " on dataset: " << SET_NAME << std::endl;

	//     if (MODEL_NAME == "MinkUNet34C" || MODEL_NAME == "PointTransformerV3")
	//       SUFFIX = "ply_xyzln";

	//     fs::path GT_PATH("/media/wd_hdd/ubuntu/datasets/complex_structure_visualization/" + SET_NAME + "/" + SUFFIX);

	//     fs::path PRED_PATH(model + "/" + SET_NAME); // DL METHODS

	//     for (const auto & entry : fs::directory_iterator(PRED_PATH))
	//     {
	//       if (entry.path().extension() == ".ply") {
	//         std::string CLOUD_NAME = entry.path().stem().string();
	//         plot_by_confusion_matrix(GT_PATH, PRED_PATH, CLOUD_NAME);
	//       }
	//     }
	//   }
	// }

	const fs::path GT_PATH("/media/wd_hdd/ubuntu/datasets/sncs_test/v1/05/pcd");
	const fs::path PRED_PATH("/media/wd_hdd/ubuntu/sncs_revision/adhoc_results/hybrid/05"); // DL METHODS
	// const fs::path PRED_PATH("/media/wd_hdd/ubuntu/sncs_revision/dl_results/05"); // DL METHODS


	// const fs::path GT_PATH("/media/arvc/data/datasets/sncs_test/v1/05/ply_xyzln");
	// const fs::path PRED_PATH("/media/arvc/data/sncs_revision/sncs_dl_results/PointNet2BinSeg/240723142940/sncs_test_unfixed/v1/05"); // DL METHODS


	if (argc == 1)
	{
		std::vector<fs::path> pred_files;

		for (const auto &entry : fs::directory_iterator(PRED_PATH))
		{
			if (entry.path().extension() == ".ply")
			{
				pred_files.push_back(entry.path());
			}
		}

		std::sort(pred_files.begin(), pred_files.end());

		for (const auto &entry : pred_files)
		{
			std::string CLOUD_NAME = entry.stem().string();
			plot_by_confusion_matrix(GT_PATH, PRED_PATH, CLOUD_NAME);
		}
	}
	else if (argc == 2)
	{
		std::string CLOUD_NAME = argv[1];
		plot_by_confusion_matrix(GT_PATH, PRED_PATH, CLOUD_NAME);
	}
	else
	{
		std::cout << "Usage: " << argv[0] << " [CLOUD_NAME]" << std::endl;
		std::cout << "If no CLOUD_NAME is provided, all clouds in the PRED_PATH will be processed." << std::endl;
	}

	return 0;
}

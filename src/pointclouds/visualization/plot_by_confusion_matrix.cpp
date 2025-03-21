#include <iostream>
#include <filesystem>
#include <thread>

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


struct ConfusionMatrixIndices {
    pcl::IndicesPtr tp_idx;
    pcl::IndicesPtr tn_idx;
    pcl::IndicesPtr fp_idx;
    pcl::IndicesPtr fn_idx;

    ConfusionMatrixIndices() : tp_idx(new pcl::Indices), tn_idx(new pcl::Indices), fp_idx(new pcl::Indices), fn_idx(new pcl::Indices) {}
};



ConfusionMatrixIndices compute_cm_indices(const pcl::Indices& gt_truss, const pcl::Indices& gt_ground, const pcl::Indices pred_truss, const pcl::Indices& pred_ground) {
    ConfusionMatrixIndices cm_indices;

    // True Positives (TP): Ground truth is truss and prediction is truss
    for (int idx : gt_truss) {
        if (std::find(pred_truss.begin(), pred_truss.end(), idx) != pred_truss.end()) {
            cm_indices.tp_idx->push_back(idx);
        }
    }

    // True Negatives (TN): Ground truth is ground and prediction is ground
    for (int idx : gt_ground) {
        if (std::find(pred_ground.begin(), pred_ground.end(), idx) != pred_ground.end()) {
            cm_indices.tn_idx->push_back(idx);
        }
    }

    // False Positives (FP): Ground truth is ground but prediction is truss
    for (int idx : gt_ground) {
        if (std::find(pred_truss.begin(), pred_truss.end(), idx) != pred_truss.end()) {
            cm_indices.fp_idx->push_back(idx);
        }
    }

    // False Negatives (FN): Ground truth is truss but prediction is ground
    for (int idx : gt_truss) {
        if (std::find(pred_ground.begin(), pred_ground.end(), idx) != pred_ground.end()) {
            cm_indices.fn_idx->push_back(idx);
        }
    }

    return cm_indices;
}




void plot_by_confusion_matrix(const fs::path GT_PATH, const fs::path PRED_PATH, const string CLOUD_NAME)
{
  fs::path current_path = fs::current_path();

  PointCloudLN::Ptr gt_cloud (new PointCloudLN);
  PointCloudL::Ptr  pred_cloud (new PointCloudL);

  PointCloud::Ptr cloud_in (new PointCloud);
  
  truss_idx gt;
  truss_idx pred;

  ConfusionMatrixIndices cm_indices;


  fs::path gt_cloud_path = GT_PATH / (CLOUD_NAME + ".ply");
  fs::path pred_cloud_path = PRED_PATH / (CLOUD_NAME + ".ply");

  gt_cloud = arvc::readPointCloud<PointLN> (gt_cloud_path);
  pred_cloud = arvc::readPointCloud<PointL>(pred_cloud_path);
  cloud_in = arvc::parseToXYZ(gt_cloud);
  
  gt = get_indices<PointLN>(gt_cloud);
  pred = get_indices<PointL>(pred_cloud);

  cm_indices = compute_cm_indices(gt.truss, gt.ground, pred.truss, pred.ground);

  int cm_size = cm_indices.tp_idx->size() + cm_indices.tn_idx->size() + cm_indices.fp_idx->size() + cm_indices.fn_idx->size();

  if (cm_size != (int) cloud_in->size())
  {
    std::cout << "Error: Confusion Matrix size is not equal to the cloud size" << std::endl;
    return;
  }

  PointCloud::Ptr tp_cloud (new PointCloud);
  PointCloud::Ptr tn_cloud (new PointCloud);
  PointCloud::Ptr fp_cloud (new PointCloud);
  PointCloud::Ptr fn_cloud (new PointCloud);
  pcl::IndicesPtr error_idx (new pcl::Indices);

  error_idx->insert(error_idx->end(), cm_indices.fp_idx->begin(), cm_indices.fp_idx->end());
  error_idx->insert(error_idx->end(), cm_indices.fn_idx->begin(), cm_indices.fn_idx->end());

  tp_cloud = arvc::extract_indices(cloud_in, cm_indices.tp_idx, false);
  tn_cloud = arvc::extract_indices(cloud_in, cm_indices.tn_idx, false);
  fp_cloud = arvc::extract_indices(cloud_in, cm_indices.fp_idx, false);
  fn_cloud = arvc::extract_indices(cloud_in, cm_indices.fn_idx, false);

  // -------------------------------- // 
  // Visualization Stuff
  // -------------------------------- //
  pcl::visualization::PCLVisualizer my_vis(pred_cloud_path.string());
  fs::path cam_params_dir = GT_PATH.parent_path();
  fs::path camera_params_path = cam_params_dir / (CLOUD_NAME + "_cam_params.txt");

  try
  {
    my_vis.loadCameraParameters(camera_params_path.string());
  }
  catch(const std::exception& e)
  {
    std::cout << "No camera parameters found" << std::endl;
  }

  // my_vis.setFullScreen(true);
  my_vis.setBackgroundColor(1,1,1);

  my_vis.addCoordinateSystem(1.5, "sensor_origin");
  auto pos = cloud_in->sensor_origin_;
  auto ori = cloud_in->sensor_orientation_;
  Eigen::Vector3f position(pos[0], pos[1], pos[2]);



  // my_vis.addCube(position, ori, 0.3, 0.3, 0.3, "sensor_origin");
  my_vis.addSphere(pcl::PointXYZ(.0,.0,.0), 0.1, 0, 0, 1, "sphere_origin");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> tp_color (tp_cloud, 76, 175, 80);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> tn_color (tn_cloud, 100,100,100);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> fp_color (fp_cloud, 211, 47, 47);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> fn_color (fn_cloud, 255, 152, 0);

  my_vis.addPointCloud(tp_cloud, tp_color, "tp_cloud");
  my_vis.addPointCloud(tn_cloud, tn_color, "tn_cloud");
  my_vis.addPointCloud(fp_cloud, fp_color, "fp_cloud");
  my_vis.addPointCloud(fn_cloud, fn_color, "fn_cloud");

  my_vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "tp_cloud");
  my_vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "tn_cloud");
  my_vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "fp_cloud");
  my_vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "fn_cloud");


  while (!my_vis.wasStopped())
  {
    my_vis.saveCameraParameters(camera_params_path.string());
    my_vis.spinOnce(100);
  }

  my_vis.saveScreenshot(PRED_PATH.string() + "/" + (CLOUD_NAME + "_conf_matrix.png"));


}

int main()
{
  std::vector<std::string> MODELS = {"PointNetBinSeg", "PointNet2BinSeg", "MinkUNet34C", "PointTransformerV3"};
  std::vector<std::string> FEATURES = {"c","nxnynz", "xyz", "xyzc", "xyznxnynz"};
  std::vector<std::string> DATASETS = {"orto", "crossed", "00", "01", "02", "03"};
  
  std::string MODEL_NAME = MODELS[3];
  std::string FEATURE = FEATURES[0];
  std::string SET_NAME = DATASETS[4];
  std::string SUFFIX = "ply_xyzln";

  // MULTIPLE MODELS AN FILES
  // for (const std::string& model : MODELS)
  // {
  //   for (const std::string& dataset : DATASETS)
  //   {
  //     MODEL_NAME = model;
  //     SET_NAME = dataset;

  //     if (MODEL_NAME == "MinkUNet34C" || MODEL_NAME == "PointTransformerV3")
  //       SUFFIX = "ply_xyzln";

  //     fs::path GT_PATH("/media/wd_hdd/ubuntu/datasets/complex_structure_visualization/" + SET_NAME + "/" + SUFFIX);
  //     fs::path PRED_PATH("/media/wd_hdd/ubuntu/datasets/complex_structure_inferences/visualization/results/" + MODEL_NAME + "/" + FEATURE + "/" + SET_NAME);

  //     for (const auto & entry : fs::directory_iterator(PRED_PATH))
  //     {
  //       if (entry.path().extension() == ".ply") {
  //         std::string CLOUD_NAME = entry.path().stem().string();
  //         plot_by_confusion_matrix(GT_PATH, PRED_PATH, CLOUD_NAME);
  //       }
  //     }
  //   }
  // }


  // SINGLE MODEL AND FILE
  if (MODEL_NAME == "MinkUNet34C" || MODEL_NAME == "PointTransformerV3")
  SUFFIX = "ply_xyzln";

  fs::path GT_PATH("/media/wd_hdd/ubuntu/datasets/complex_structure_visualization/" + SET_NAME + "/" + SUFFIX);
  fs::path PRED_PATH("/media/wd_hdd/ubuntu/datasets/complex_structure_inferences/dl/results/" + MODEL_NAME + "/" + FEATURE + "/" + SET_NAME); // DL METHODS
  // fs::path PRED_PATH("/media/wd_hdd/ubuntu/datasets/complex_structure_inferences/analytical/results/" + SET_NAME);  // ANALYTICAL METHODS

  
  for (const auto & entry : fs::directory_iterator(PRED_PATH))
  {
    if (entry.path().extension() == ".ply") {
      std::string CLOUD_NAME = entry.path().stem().string();
      plot_by_confusion_matrix(GT_PATH, PRED_PATH, CLOUD_NAME);
    }
  }

  return 0;  
}



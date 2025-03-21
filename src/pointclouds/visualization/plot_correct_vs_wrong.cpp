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
#include <yaml-cpp/yaml.h>

using namespace std;
namespace fs = std::filesystem;

int main(int argc, char **argv)
{

    // Manage dynamic paths
    const fs::path CURRENT_PATH = fs::current_path();
    const std::string SET_ID = CURRENT_PATH.parent_path().filename().string(); 

    std::string GT_SUFFIX;
    if (CURRENT_PATH.parent_path().filename().string() == "PointNetBinSeg")
        GT_SUFFIX = "ply_xyzln_fixedSize";
    else
        GT_SUFFIX = "ply_xyzln";

    YAML::Node config = YAML::LoadFile("/home/arvc/workspaces/code_ws/config/config.yaml");
    const fs::path GT_PATH = config["GT_PATH"].as<string>();

    const fs::path GT_PATH_FULL = GT_PATH / GT_SUFFIX;


    // Create point cloud objects
    PointCloud::Ptr gt_cloud_xyz(new PointCloud);
    PointCloudLN::Ptr gt_cloud(new PointCloudLN);
    PointCloudL::Ptr pred_cloud(new PointCloudL);

    gt_indices _gt_idx;
    gt_indices _i_idx;
    cm_indices _cm_indices;
    _cm_indices.tp_idx.reset(new pcl::Indices);
    _cm_indices.tn_idx.reset(new pcl::Indices);
    _cm_indices.fp_idx.reset(new pcl::Indices);
    _cm_indices.fn_idx.reset(new pcl::Indices);


    if (argc < 2)
    {
        std::vector<fs::path> pred_files;
        for (const auto &entry : fs::directory_iterator(CURRENT_PATH))
        {
            if (entry.path().extension() == ".ply" || entry.path().extension() == ".pcd")
            pred_files.push_back(entry.path());
        }

        std::sort(pred_files.begin(), pred_files.end(), [](const fs::path &a, const fs::path &b) {
            return a.filename().string() < b.filename().string();
        });

        
        for (const fs::path PRED_FILE : pred_files) {

            const fs::path gt_file = GT_PATH_FULL / PRED_FILE.filename();
            
            std::cout << "PRED_FILE: " << PRED_FILE << std::endl;
            std::cout << "GT_FILE: " << gt_file << std::endl;

            assert(fs::exists(PRED_FILE));
            assert(fs::exists(gt_file));

            // Read the point clouds
            pred_cloud = arvc::readPointCloud<PointL>(PRED_FILE);
            gt_cloud = arvc::readPointCloud<PointLN>(gt_file.string());
            gt_cloud_xyz = arvc::parseToXYZ(gt_cloud);

            // Get the indices of the ground truth and the predictions
            _gt_idx = arvc::getGroundTruthIndices(gt_cloud);
            _i_idx = arvc::getGroundTruthIndices(pred_cloud);
            _cm_indices = arvc::compute_cm_indices(_gt_idx.truss, _gt_idx.ground, _i_idx.truss, _i_idx.ground);
    
            PointCloud::Ptr error_cloud(new PointCloud);
            PointCloud::Ptr truss_cloud(new PointCloud);
            PointCloud::Ptr ground_cloud(new PointCloud);
            pcl::IndicesPtr error_idx(new pcl::Indices);
    
            error_idx->insert(error_idx->end(), _cm_indices.fp_idx->begin(), _cm_indices.fp_idx->end());
            error_idx->insert(error_idx->end(), _cm_indices.fn_idx->begin(), _cm_indices.fn_idx->end());
    
            truss_cloud = arvc::extract_indices(gt_cloud_xyz, _cm_indices.tp_idx, false);
            ground_cloud = arvc::extract_indices(gt_cloud_xyz, _cm_indices.tn_idx, false);
            error_cloud = arvc::extract_indices(gt_cloud_xyz, error_idx, false);
    
            pcl::visualization::PCLVisualizer my_vis;
    
            fs::path camera_params_path = PRED_FILE.parent_path() / (PRED_FILE.stem().string() + "_camera_params.txt");
    
            try {
                my_vis.loadCameraParameters(camera_params_path.string());
            }
            catch (const std::exception &e) {}
    
            my_vis.setBackgroundColor(1, 1, 1);
            my_vis.addCoordinateSystem(0.8, "sensor_origin");
            // auto pos = gt_cloud_xyz->sensor_origin_;
            // auto ori = gt_cloud_xyz->sensor_orientation_;
    
            // Eigen::Vector3f position(pos[0], pos[1], pos[2]);
            // my_vis.addCube(position, ori, 0.3, 0.3, 0.3, "sensor_origin");
    
            pcl::visualization::PointCloudColorHandlerCustom<PointT> truss_color(truss_cloud, 50, 190, 50);
            pcl::visualization::PointCloudColorHandlerCustom<PointT> ground_color(ground_cloud, 100, 100, 100);
            pcl::visualization::PointCloudColorHandlerCustom<PointT> error_color(error_cloud, 200, 10, 10);
    
            my_vis.addPointCloud(truss_cloud, truss_color, "truss_cloud");
            my_vis.addPointCloud(ground_cloud, ground_color, "wrong_cloud");
            my_vis.addPointCloud(error_cloud, error_color, "error_cloud");
    
            while (!my_vis.wasStopped())
            {
                my_vis.saveCameraParameters(camera_params_path.string());
                my_vis.spinOnce(100);
            }
        }


    }
    else if (argc == 2)
    {
        std::string CLOUD_FILENAME = argv[1];
        const fs::path PRED_FILE = CURRENT_PATH / CLOUD_FILENAME;

        const fs::path gt_file = GT_PATH_FULL / PRED_FILE.filename();
        
        std::cout << "PRED_FILE: " << PRED_FILE << std::endl;
        std::cout << "GT_FILE: " << gt_file << std::endl;


        assert(fs::exists(PRED_FILE));
        assert(fs::exists(gt_file));

        // Read the point clouds
        pred_cloud = arvc::readPointCloud<PointL>(PRED_FILE);
        gt_cloud = arvc::readPointCloud<PointLN>(gt_file.string());
        gt_cloud_xyz = arvc::parseToXYZ(gt_cloud);

        // Get the indices of the ground truth and the predictions
        _gt_idx = arvc::getGroundTruthIndices(gt_cloud);
        _i_idx = arvc::getGroundTruthIndices(pred_cloud);
        _cm_indices = arvc::compute_cm_indices(_gt_idx.truss, _gt_idx.ground, _i_idx.truss, _i_idx.ground);

        PointCloud::Ptr error_cloud(new PointCloud);
        PointCloud::Ptr truss_cloud(new PointCloud);
        PointCloud::Ptr ground_cloud(new PointCloud);
        pcl::IndicesPtr error_idx(new pcl::Indices);

        error_idx->insert(error_idx->end(), _cm_indices.fp_idx->begin(), _cm_indices.fp_idx->end());
        error_idx->insert(error_idx->end(), _cm_indices.fn_idx->begin(), _cm_indices.fn_idx->end());

        truss_cloud = arvc::extract_indices(gt_cloud_xyz, _cm_indices.tp_idx, false);
        ground_cloud = arvc::extract_indices(gt_cloud_xyz, _cm_indices.tn_idx, false);
        error_cloud = arvc::extract_indices(gt_cloud_xyz, error_idx, false);

        pcl::visualization::PCLVisualizer my_vis;

        fs::path camera_params_path = PRED_FILE.parent_path() / (PRED_FILE.stem().string() + "_camera_params.txt");

        try {
            my_vis.loadCameraParameters(camera_params_path.string());
        }
        catch (const std::exception &e) {}

        my_vis.setBackgroundColor(1, 1, 1);
        my_vis.addCoordinateSystem(0.8, "sensor_origin");
        // auto pos = gt_cloud_xyz->sensor_origin_;
        // auto ori = gt_cloud_xyz->sensor_orientation_;

        // Eigen::Vector3f position(pos[0], pos[1], pos[2]);
        // my_vis.addCube(position, ori, 0.3, 0.3, 0.3, "sensor_origin");

        pcl::visualization::PointCloudColorHandlerCustom<PointT> truss_color(truss_cloud, 50, 190, 50);
        pcl::visualization::PointCloudColorHandlerCustom<PointT> ground_color(ground_cloud, 100, 100, 100);
        pcl::visualization::PointCloudColorHandlerCustom<PointT> error_color(error_cloud, 200, 10, 10);

        my_vis.addPointCloud(truss_cloud, truss_color, "truss_cloud");
        my_vis.addPointCloud(ground_cloud, ground_color, "wrong_cloud");
        my_vis.addPointCloud(error_cloud, error_color, "error_cloud");

        while (!my_vis.wasStopped())
        {
            my_vis.saveCameraParameters(camera_params_path.string());
            my_vis.spinOnce(100);
        }
    }
    else
    {
        std::cout << "Usage: arvc_plot_correct_vs_wrong <cloud_name>" << std::endl;
        std::cout << "or" << std::endl;
        std::cout << "Move to the directory where the clouds are located:" << std::endl;
        std::cout << "Usage: arvc_plot_correct_vs_wrong" << std::endl;
    }

    return 0;
}

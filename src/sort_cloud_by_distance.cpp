
#include <iostream>
#include <chrono>

#include "pointclouds/read_cloud.hpp"
#include "pointclouds/write_cloud.hpp"
#include "pointclouds/transforms/serialise.hpp"
#include "tqdm.hpp"

#include <pcl/filters/filter.h>
#include <pcl/common/common.h>

int main(int argc, char **argv)
{
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    const std::string TARGET_DIR_NAME = "ply_xyzln_fixedSize";
    const std::string OUTPUT_DIR_NAME = "ply_xyzln_fixedSize_sorted";
    typedef pcl::PointXYZLNormal PointIN;
    typedef pcl::PointXYZLNormal PointOUT;

    pcl::PointCloud<PointIN>::Ptr cloud_in(new pcl::PointCloud<PointIN>);
    pcl::PointCloud<PointOUT>::Ptr cloud_out(new pcl::PointCloud<PointOUT>);

    fs::path current_dir = fs::current_path();

    if (argc < 2)
    {
        std::vector<fs::path> cloud_paths;
        for (const auto &entry : fs::directory_iterator(current_dir))
        {
            // if(entry.path().extension() == ".pcd" || entry.path().extension() == ".ply")
            //   path_vector.push_back(entry.path());

            // Entirely retTruss dataset
            if (entry.is_directory())
            {
                fs::path set_dir = entry.path() / TARGET_DIR_NAME;
                std::cout << "Getting clouds from set: " << set_dir << std::endl;

                for (const auto &cloud_entry : fs::directory_iterator(set_dir))
                {
                    if (cloud_entry.path().extension().string() == ".pcd" || cloud_entry.path().extension().string() == ".ply")
                    {
                        cloud_paths.push_back(cloud_entry.path());
                    }
                }
            }
        }

        std::cout << "Reading clouds..." << std::endl;
        for (const fs::path &cloud_entry : tq::tqdm(cloud_paths))
        {
            cloud_in = readPointCloud<PointIN>(cloud_entry.string());

            pcl::Indices uselsess;
            pcl::removeNaNFromPointCloud<PointIN>(*cloud_in, *cloud_in, uselsess);
            pcl::removeNaNNormalsFromPointCloud<PointIN>(*cloud_in, *cloud_in, uselsess);

            cloud_out = sortCloudByDistance<PointIN>(cloud_in);

            writeCloud<PointOUT>(cloud_out, cloud_entry, OUTPUT_DIR_NAME);
        }

        return 0;
    }
}
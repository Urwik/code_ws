#include "pointclouds/filters/random_downsample_cloud.hpp"
#include "pointclouds/read_cloud.hpp"
#include "pointclouds/write_cloud.hpp"
#include "pointclouds/features/normal_estimation.hpp"
#include "pointclouds/filters/farthest_point_downsample.hpp"
#include "pointclouds/visualization/view_cloud_normals.hpp"
#include "tqdm.hpp"

#include <iostream>
#include <filesystem>
#include <time.h>

#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>

#include <thread>
#include <vector>
#include <unordered_map>

using namespace std;

namespace fs = std::filesystem;

void writeDiscardedClouds(std::vector<fs::path> &discarded_clouds)
{
    fs::path abs_file_path = fs::current_path() / "discarded_clouds.txt";

    std::cout << "Writing discarded clouds to " << abs_file_path << std::endl;

    std::ofstream file(abs_file_path);
    for (const fs::path &cloud_path : discarded_clouds)
    {
        file << cloud_path.string() << std::endl;
    }
    file.close();
}

int main(int argc, char **argv)
{
    const std::string TARGET_DIR_NAME = "ply_xyzln";
    const std::string OUTPUT_DIR_NAME = "ply_xyzln_fixedSize";
    const int FIXED_NUM_POINTS = 20000;
    typedef pcl::PointXYZLNormal PointIN;
    typedef pcl::PointXYZLNormal PointOUT;

    
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    pcl::PointCloud<PointIN>::Ptr cloud_in(new pcl::PointCloud<PointIN>);
    pcl::PointCloud<PointOUT>::Ptr cloud_out(new pcl::PointCloud<PointOUT>);

    fs::path current_dir = fs::current_path();

    std::vector<fs::path> discarded_clouds;
    std::vector<fs::path> accepted_clouds;


    //--------------------------------------------------------------------------------//
    // FOR ALL CLOUDS IN CURRENT DIRECTORY                                            //
    //--------------------------------------------------------------------------------//
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
            else if (entry.is_regular_file()) {
                if (entry.path().extension().string() == ".pcd" || entry.path().extension().string() == ".ply")
                    cloud_paths.push_back(entry.path());
            }
        }


        std::cout << "Reading clouds..." << std::endl;
        for(const fs::path &cloud_entry : tq::tqdm(cloud_paths))
        {
          cloud_in = readPointCloud<PointIN>(cloud_entry.string());

          pcl::Indices uselsess;
          pcl::removeNaNFromPointCloud<PointIN>(*cloud_in, *cloud_in, uselsess);
          pcl::removeNaNNormalsFromPointCloud<PointIN>(*cloud_in, *cloud_in, uselsess);

          int cloud_size = cloud_in->points.size();

          if (cloud_size < FIXED_NUM_POINTS) {
            // std::cout << std::endl << "Cloud " << cloud_entry.string() << " has less than " << FIXED_NUM_POINTS << " points. Discarding..." << std::endl;
            discarded_clouds.push_back(cloud_entry);
          }
          else
          {
            accepted_clouds.push_back(cloud_entry);
            cloud_out = randomSampleCloud(cloud_in, FIXED_NUM_POINTS);
            writeCloud<PointOUT>(cloud_out, cloud_entry, OUTPUT_DIR_NAME);
          }
        }

        writeDiscardedClouds(discarded_clouds);
    }

    //--------------------------------------------------------------------------------//
    // ONLY ONE CLOUD                                                                 //
    //--------------------------------------------------------------------------------//
    else
    {

        std::cout << "NOT IMPLEMENTED..." << std::endl;
    }

    std::cout << "COMPLETED!!" << std::endl;
    return 0;
}

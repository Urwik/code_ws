// #include "pointclouds/arvc_utils.cpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "arvc_utils.hpp"

#include <ignition/math/Matrix4.hh>
#include <ignition/math.hh>
#include <eigen3/Eigen/Dense>


namespace im = ignition::math;

class truss_plane_detector
{

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz;
    im::Matrix4d QSensorToTruss;

public:
    truss_plane_detector();
    ~truss_plane_detector();

    void read_cloud();
    void detect_plane();
};

truss_plane_detector::truss_plane_detector()
{
    this->cloud_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->QSensorToTruss = im::Matrix4d::Zero;
}

truss_plane_detector::~truss_plane_detector()
{
    this->cloud_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->QSensorToTruss = im::Matrix4d::Zero;
}

void truss_plane_detector::detect_plane(){
    
    pcl::ModelCoefficientsPtr coefficients(new pcl::ModelCoefficients);
    bool optimize_coefficients = true;
    double distance_threshold = 0.01;
    int max_iterations = 1000;

    coefficients = arvc::compute_planar_ransac( this->cloud_xyz,
                                                optimize_coefficients,
                                                distance_threshold,
                                                max_iterations);

}

int main(int argc, char const *argv[])
{
    /* code */
    return 0;
}

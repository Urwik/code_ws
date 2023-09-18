// #include "pointclouds/arvc_utils.cpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ignition/math/Matrix4.hh>
#include <eigen3/Eigen/Dense>


namespace im = ignition::math;

class truss_plane_detector
{

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz;
    const im::Matrix4d &QSensorToTruss;
public:
    truss_plane_detector();
    ~truss_plane_detector();
};

truss_plane_detector::truss_plane_detector(/* args */)
{
    this->cloud_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->QSensorToTruss = im::Matrix4d::Zero;

}

truss_plane_detector::~truss_plane_detector()
{
}

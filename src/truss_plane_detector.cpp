#include "arvc_utils.hpp"

#include <ignition/math/Matrix4.hh>
#include <ignition/math.hh>
#include <pcl/pcl_config.h>


namespace im = ignition::math;
namespace eig = Eigen;


class truss_plane_detector
{

public:
    truss_plane_detector();
    truss_plane_detector(double range_dist, im::Pose3d QSensorToTruss);
    ~truss_plane_detector();

    void read_cloud();
    void detect_plane();
    void get_close_points();

    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;
    im::Pose3d QSensorToTruss;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_xyz;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_search_xyz;
    arvc::plane plane;
};

// DEFAULT CONSTRUCTOR
truss_plane_detector::truss_plane_detector()
{
    this->cloud_in_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->cloud_search_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->QSensorToTruss = im::Pose3d::Zero;
    this->min_pt = Eigen::Vector4f::Zero();
    this->max_pt = Eigen::Vector4f::Zero();
}

// CONSTRUCTOR
truss_plane_detector::truss_plane_detector(double range_dist, im::Pose3d QSensorToTruss)
{
    this->cloud_in_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->cloud_search_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->QSensorToTruss = QSensorToTruss;
    this->min_pt << -range_dist, -range_dist, -range_dist, 1.0;
    this->max_pt << range_dist, range_dist, range_dist, 1.0;
}

// DESTRUCTOR
truss_plane_detector::~truss_plane_detector()
{
    this->cloud_in_xyz.reset();
    this->cloud_search_xyz.reset();
}


void truss_plane_detector::detect_plane(){
    
    pcl::PointIndicesPtr point_indices (new pcl::PointIndices);
    pcl::SACSegmentation<PointT> ransac;
    pcl::ModelCoefficientsPtr plane_coeffs (new pcl::ModelCoefficients);

    ransac.setInputCloud(this->cloud_search_xyz);
    ransac.setOptimizeCoefficients(false);
    ransac.setModelType(pcl::SACMODEL_PLANE);
    ransac.setMethodType(pcl::SAC_RANSAC);
    ransac.setMaxIterations(1000);
    ransac.setDistanceThreshold(0.02);
    ransac.segment(*point_indices, *plane_coeffs);
    
    *this->plane.coeffs = *plane_coeffs;
    *this->plane.inliers = *point_indices;
}


void truss_plane_detector::get_close_points(){
    
    pcl::IndicesPtr indices(new pcl::Indices);

    pcl::getPointsInBox(*this->cloud_in_xyz, this->min_pt, this->max_pt, *indices);
    
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    
    extract.setInputCloud(this->cloud_in_xyz);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*this->cloud_search_xyz);

}


int main(int argc, char const *argv[])
{
    im::Pose3d TrussSensorPose;
    TrussSensorPose.Pos().Set(0.0, 0.0, 0.0);
    TrussSensorPose.Rot().Euler(im::Vector3d(0.0, 0.0, 0.0));

    double range_dist = 1;

    truss_plane_detector td(range_dist, TrussSensorPose);

    td.cloud_in_xyz = arvc::readCloud("/media/arvc/data/datasets/ARVCTRUSS/valid/ply_xyzlabelnormal/00000.ply");
    td.cloud_in_xyz = arvc::remove_origin_points(td.cloud_in_xyz);
    td.get_close_points();
    td.detect_plane();

    PointCloud::Ptr plane_cloud = td.plane.getCloud(td.cloud_search_xyz);

    // arvc::visualizeClouds(td.cloud_search_xyz, plane_cloud);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    viewer->setBackgroundColor(0.5, 0.5, 0.5);
    viewer->addCoordinateSystem(0.50, "global");

    Eigen::Vector3f position(0.0, 0.0, 0.0);
    pcl::PointXYZ position_point(0,0,0);

    // viewer->addCube(position, td.cloud_search_xyz->sensor_orientation_, 0.085, 0.085, 0.073, "cube");
    viewer->addSphere(position_point, 0.05, "sphere");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "sphere");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "sphere");

    viewer->addPointCloud<pcl::PointXYZ>(td.cloud_search_xyz, "cloud");
    pcl::PointXYZ centroid;
    pcl::computeCentroid(*plane_cloud, centroid);

    pcl::PointXYZ plane_normal;
    plane_normal.x = td.plane.coeffs->values[0] + centroid.x;
    plane_normal.y = td.plane.coeffs->values[1] + centroid.y;
    plane_normal.z = td.plane.coeffs->values[2] + centroid.z;


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> plane_color(plane_cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(plane_cloud, plane_color, "plane");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "plane");
    viewer->addArrow<pcl::PointXYZ, pcl::PointXYZ>(centroid, plane_normal, 0.2, 0.2, 0.2, "normal");
    

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }



    return 0;
}

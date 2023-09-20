#include "arvc_utils.hpp"

#include <ignition/math/Matrix4.hh>
#include <ignition/math.hh>
#include <pcl/pcl_config.h>

#include <Eigen/Geometry>


namespace im = ignition::math;


class truss_plane_detector
{

public:

    // DEFAULT CONSTRUCTOR
    truss_plane_detector()
    {
        this->cloud_in_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
        this->cloud_search_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
        this->QSensorToTruss = im::Pose3d::Zero;
        this->min_pt = Eigen::Vector4f::Zero();
        this->max_pt = Eigen::Vector4f::Zero();
    }

    // CONSTRUCTOR
    truss_plane_detector(double range_dist, im::Pose3d QSensorToTruss)
    {
        this->cloud_in_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
        this->cloud_search_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
        this->QSensorToTruss = QSensorToTruss;
        this->min_pt << -range_dist, -range_dist, -range_dist, 1.0;
        this->max_pt << range_dist, range_dist, range_dist, 1.0;
    }

    // DESTRUCTOR
    ~truss_plane_detector()
    {
        this->cloud_in_xyz.reset();
        this->cloud_search_xyz.reset();
    }



    void detect_plane(){
        
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


    void get_close_points(){
        
        pcl::IndicesPtr indices(new pcl::Indices);

        pcl::getPointsInBox(*this->cloud_in_xyz, this->min_pt, this->max_pt, *indices);
        
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        
        extract.setInputCloud(this->cloud_in_xyz);
        extract.setIndices(indices);
        extract.setNegative(false);
        extract.filter(*this->cloud_search_xyz);

    }


    void vector_base_transform(Eigen::Vector3d vector){

        Eigen::Vector3d vector_transformed;
        vector_transformed = this->RobotBaseToSensor.rotation() * vector;
        std::cout << "Vector transformed: " << vector_transformed << std::endl;
        
        this->second_direction = vector_transformed;
    }

    void compute_third_direction()
    {
        this->third_direction = this->basis.cross(this->second_direction);
    }

    void view_result()
    {
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

        viewer->setBackgroundColor(0.5, 0.5, 0.5);
        viewer->addCoordinateSystem(0.50, "global");

        pcl::PointXYZ position_point(0,0,0);

        viewer->addSphere(position_point, 0.05, "sphere");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "sphere");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "sphere");

        viewer->addPointCloud<pcl::PointXYZ>(this->cloud_search_xyz, "cloud");


        PointCloud::Ptr plane_cloud = this->plane.getCloud(this->cloud_search_xyz);
        pcl::PointXYZ centroid;
        pcl::computeCentroid(*plane_cloud, centroid);

        pcl::PointXYZ plane_normal;
        plane_normal.x = this->plane.coeffs->values[0] + centroid.x;
        plane_normal.y = this->plane.coeffs->values[1] + centroid.y;
        plane_normal.z = this->plane.coeffs->values[2] + centroid.z;


        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> plane_color(plane_cloud, 0, 255, 0);
        viewer->addPointCloud<pcl::PointXYZ>(plane_cloud, plane_color, "plane");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "plane");
        viewer->addArrow<pcl::PointXYZ, pcl::PointXYZ>(centroid, plane_normal, 0.2, 0.2, 0.2, "normal");
        
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
        }

    }


    // PUBLIC ATTRIBUTES
    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;
    im::Pose3d QSensorToTruss;
    Eigen::Transform <double, 3, Eigen::Affine> RobotBaseToSensor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_xyz;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_search_xyz;
    arvc::plane plane;
    Eigen::Vector3d basis;
    Eigen::Vector3d second_direction;
    Eigen::Vector3d third_direction;

};

int main(int argc, char const *argv[])
{
    im::Pose3d TrussSensorPose;
    TrussSensorPose.Pos().Set(0.0, 0.0, 0.0);
    TrussSensorPose.Rot().Euler(im::Vector3d(0.0, 0.0, 0.0));

    double range_dist = 1;

    truss_plane_detector td(range_dist, TrussSensorPose);

    td.cloud_in_xyz = arvc::readCloud("/home/fran/datasets/test_visualization/pcd/00000.pcd");
    td.cloud_in_xyz = arvc::remove_origin_points(td.cloud_in_xyz);
    td.get_close_points();
    td.detect_plane();
    td.view_result();


    return 0;
}

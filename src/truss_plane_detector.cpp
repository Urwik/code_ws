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
        
        this->range_dist = 1;
        this->RobotBaseToSensorTF = Eigen::Affine3f::Identity();
        this->basis = Eigen::Vector3f(0,0,1);
        this->viewer.reset(new pcl::visualization::PCLVisualizer("viewer"));
    }

    // CONSTRUCTOR
    truss_plane_detector(double _range_dist)
    {
        this->cloud_in_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
        this->cloud_search_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);

        this->range_dist = _range_dist;
        this->RobotBaseToSensorTF = Eigen::Affine3f::Identity();
        this->basis = Eigen::Vector3f(0,0,1);
        this->viewer.reset(new pcl::visualization::PCLVisualizer("viewer"));
    }

    // DESTRUCTOR
    ~truss_plane_detector()
    {
        this->cloud_in_xyz.reset();
        this->cloud_search_xyz.reset();
        this->viewer.reset();
    }


    void detect_plane(){
        
        pcl::PointIndicesPtr point_indices (new pcl::PointIndices);
        pcl::SACSegmentation<PointT> ransac;
        pcl::ModelCoefficientsPtr plane_coeffs (new pcl::ModelCoefficients);

        ransac.setInputCloud(this->cloud_search_xyz);
        ransac.setOptimizeCoefficients(true);
        ransac.setModelType(pcl::SACMODEL_PLANE);
        ransac.setMethodType(pcl::SAC_RANSAC);
        ransac.setMaxIterations(1000);
        ransac.setDistanceThreshold(0.02);
        ransac.segment(*point_indices, *plane_coeffs);
        
        *this->plane.coeffs = *plane_coeffs;
        *this->plane.inliers = *point_indices;
        this->plane.normal = Eigen::Vector3f(plane_coeffs->values[0], plane_coeffs->values[1], plane_coeffs->values[2]);
        this->add_indices_to_view(this->plane.inliers);

        arvc::remove_indices_from_cloud(this->cloud_search_xyz, this->plane.inliers);
    }


    arvc::plane detect_plane(Eigen::Vector3f direction){
        
        pcl::SACSegmentation<PointT> ransac;
        arvc::plane plane;

        ransac.setInputCloud(this->cloud_search_xyz);
        ransac.setOptimizeCoefficients(true);
        ransac.setModelType(pcl::SACMODEL_PLANE);
        ransac.setMethodType(pcl::SAC_RANSAC);
        ransac.setMaxIterations(1000);
        ransac.setDistanceThreshold(0.02);
        ransac.setAxis(direction);
        ransac.segment(*plane.inliers, *plane.coeffs);


        if (plane.inliers->indices.size() > 500)
        {
            cout << "Plane found" << endl;
            cout << "Plane inliers size: " << plane.inliers->indices.size() << endl;
            this->add_indices_to_view(plane.inliers);
            arvc::remove_indices_from_cloud(this->cloud_search_xyz, plane.inliers);
            return plane;
        }
        else
        {
            cout << "Plane not found" << endl;
        }

        
    }


    void get_close_points(){
        
        pcl::IndicesPtr indices(new pcl::Indices);
        const float range = this->range_dist;

        Eigen::Vector4f min_pt(-range_dist, -range_dist, -range_dist, 1.0);
        Eigen::Vector4f max_pt(range_dist, range_dist, range_dist, 1.0);

        pcl::getPointsInBox(*this->cloud_in_xyz, min_pt, max_pt, *indices);
        
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        
        extract.setInputCloud(this->cloud_in_xyz);
        extract.setIndices(indices);
        extract.setNegative(false);
        extract.filter(*this->cloud_search_xyz);

    }


    void second_direction_to_base(Eigen::Vector3f vector){
        this->second_direction = this->RobotBaseToSensorTF.linear() * vector;
    }


    void compute_third_direction(){
        this->third_direction = this->basis.cross(this->second_direction);
    }


    void view_result()
    {
        this->viewer->setBackgroundColor(0.5, 0.5, 0.5);

        if (fs::exists("camera.txt"))
            this->viewer->loadCameraParameters("camera.txt");

        this->add_sensor_origin();
        this->add_plane_normal();

        this->draw_search_directions();
        this->hold_view();
    }


    void add_sensor_origin()
    {

        // ROBOT BASE COORDINATE SYSTEM
        this->viewer->addCoordinateSystem(0.1, "origin");
        pcl::PointXYZ origin_point(0,0,0);
        this->viewer->addSphere(origin_point, 0.02, "sphere");
        this->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "sphere");
        this->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "sphere");
        this->viewer->addText3D("os_0", origin_point, 0.02, 1.0, 1.0, 1.0, "origin_text");

        // SENSOR COORDINATE SYSTEM 
        this->viewer->addCoordinateSystem(0.1, this->RobotBaseToSensorTF, "sensor");
        pcl::PointXYZ sensor_origin_point;
        sensor_origin_point.x = this->RobotBaseToSensorTF.translation().x();
        sensor_origin_point.y = this->RobotBaseToSensorTF.translation().y();
        sensor_origin_point.z = this->RobotBaseToSensorTF.translation().z();
        this->viewer->addSphere(sensor_origin_point, 0.02, "sphere2");
        this->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "sphere2");
        this->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "sphere2");
        this->viewer->addText3D("os_sensor", sensor_origin_point, 0.02, 1.0, 1.0, 1.0, "sensor_origin_text");

        /*      
        Eigen::Vector3f vector_x;
        vector_x = this->RobotBaseToSensorTF * Eigen::Vector3f(5,0,0);
        pcl::PointXYZ x_vector(vector_x[0], vector_x[1], vector_x[2]);
        this->viewer->addLine(sensor_origin_point, x_vector, 1.0, 0.0, 0.0, "line_x"); 
        */
       
    }


    void add_plane_normal()
    {
        this->viewer->addPointCloud<pcl::PointXYZ>(this->cloud_search_xyz, "cloud");
        PointCloud::Ptr plane_cloud = this->plane.getCloud(this->cloud_search_xyz);
        pcl::PointXYZ centroid;
        pcl::computeCentroid(*plane_cloud, centroid);

        // CREATE THE NORMAL VECTOR IN EIGEN FORMAT
        Eigen::Vector3f normal(this->plane.coeffs->values[0], this->plane.coeffs->values[1], this->plane.coeffs->values[2]);
        Eigen::Translation3f translation(centroid.x, centroid.y, centroid.z);

        // MAKE THE NORMAL VECTOR SMALLER        
        normal = -normal / 5;
        normal = translation * normal;
        this->normal = normal;

        // SAVE THE NORMAL VECTOR IN A PCL FORMAT
        pcl::PointXYZ plane_normal;
        plane_normal.x = normal[0];
        plane_normal.y = normal[1];
        plane_normal.z = normal[2];

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> plane_color(plane_cloud, 0, 255, 0);
        this->viewer->addPointCloud<pcl::PointXYZ>(plane_cloud, plane_color, "plane_points");
        this->viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "plane_points");
        
        this->viewer->addArrow<pcl::PointXYZ, pcl::PointXYZ>(plane_normal, centroid, 0.0, 0.0, 1.0, false, "normal");
        this->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 10, "normal");

        this->viewer->addPlane(*this->plane.coeffs, centroid.x, centroid.y, centroid.z, "plane");
        this->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.5, "plane");
        this->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "plane");

    }


    void add_plane_to_view(const arvc::plane& _plane){

        if (_plane.inliers->indices.size() < 100)
            return;

        pcl::PointXYZ centroid;
        pcl::computeCentroid(*this->cloud_search_xyz, _plane.inliers->indices, centroid);

        this->ss.str("");
        this->ss << "plane" << this->counter;

        this->viewer->addPlane(*_plane.coeffs, centroid.x, centroid.y, centroid.z, this->ss.str());
        this->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, this->ss.str());
        this->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, this->ss.str());
    }


    void add_indices_to_view(pcl::PointIndicesPtr& indices){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*this->cloud_search_xyz, *indices, *cloud);
        this->ss.str("");
        this->ss << "cloud" << this->counter;

        this->viewer->addPointCloud<pcl::PointXYZ>(cloud, this->ss.str());
    }
    void hold_view()
    {
        while (!this->viewer->wasStopped())
            this->viewer->spinOnce(100);
    }


    void draw_search_directions()
    {
        pcl::PointXYZ origin(0,0,0);

        pcl::PointXYZ x_vector(this->second_direction[0], this->second_direction[1], this->second_direction[2]);
        pcl::PointXYZ y_vector(this->third_direction[0], this->third_direction[1], this->third_direction[2]);
        pcl::PointXYZ z_vector(this->basis[0], this->basis[1], this->basis[2]);

        viewer->addArrow<pcl::PointXYZ, pcl::PointXYZ>(x_vector, origin, 1.0, 0.0, 0.0, false, "x_direction");
        viewer->addArrow<pcl::PointXYZ, pcl::PointXYZ>(y_vector, origin, 0.0, 1.0, 0.0, false, "y_direction");
        viewer->addArrow<pcl::PointXYZ, pcl::PointXYZ>(z_vector, origin, 0.0, 0.0, 1.0, false, "z_direction");

    }
    
    
    // PUBLIC ATTRIBUTES
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_xyz;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_search_xyz;

    Eigen::Affine3f RobotBaseToSensorTF;
    float range_dist;

    arvc::plane plane;

    Eigen::Vector3f basis;
    Eigen::Vector3f second_direction;
    Eigen::Vector3f third_direction;

    Eigen::Vector3f normal;
    int counter = 0;
    stringstream ss;

    private:
    pcl::visualization::PCLVisualizer::Ptr viewer;

};


int main(int argc, char const *argv[])
{
    const double RANGE_DIST = 1;

    truss_plane_detector td(RANGE_DIST);

    td.RobotBaseToSensorTF.translation() = Eigen::Vector3f(0.1, 0.0, 0.3);
    td.RobotBaseToSensorTF.linear() = Eigen::Quaternionf::Identity().toRotationMatrix();

    

    // td.cloud_in_xyz = arvc::readCloud("/home/fran/datasets/test_visualization/pcd/00000.pcd");
    td.cloud_in_xyz = arvc::readCloud("/media/arvc/data/datasets/ARVCTRUSS/valid/ply_xyzlabelnormal/00000.ply");
    td.cloud_in_xyz = arvc::remove_origin_points(td.cloud_in_xyz);
    td.get_close_points();
    pcl::transformPointCloud(*td.cloud_search_xyz, *td.cloud_search_xyz, td.RobotBaseToSensorTF);

    td.detect_plane();
    td.second_direction_to_base(td.plane.normal);
    td.compute_third_direction();

    // do
    // {
    //     arvc::plane detected_plane = td.detect_plane(-td.second_direction);
    //     cout << "Detected plane inliers size: " << detected_plane.inliers->indices.size() << endl;

    //     td.counter++;
    //     td.add_plane_to_view(detected_plane);
    // // } while (detected_plane.inliers->indices.size() > 0);
    // } while (td.counter < 5);
    
    


    td.view_result();

    return 0;
}



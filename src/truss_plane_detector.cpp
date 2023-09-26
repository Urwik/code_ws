#include "arvc_utils.hpp"

#include <ignition/math/Matrix4.hh>
#include <ignition/math.hh>
#include <pcl/pcl_config.h>
#include <pcl/common/angles.h>

#include <Eigen/Geometry>


namespace im = ignition::math;


class truss_plane_detector
{
    struct direction
    {
        int index;
        arvc::color color;
    };
    

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
        this->v1=(0);
        this->v2=(1);
        this->viewer->createViewPort(0.0, 0.0, 0.5, 1.0, this->v1);
        this->viewer->createViewPort(0.5, 0.0, 1.0, 1.0, this->v2);
        this->search_directions.resize(3);
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
        this->v1=(0);
        this->v2=(1);
        this->viewer->createViewPort(0.0, 0.0, 0.5, 1.0, this->v1);
        this->viewer->createViewPort(0.5, 0.0, 1.0, 1.0, this->v2);
        this->search_directions.resize(3);
    }

    // DESTRUCTOR
    ~truss_plane_detector()
    {
        this->cloud_in_xyz.reset();
        this->cloud_search_xyz.reset();
        this->viewer.reset();
    }


    void detect_initial_plane(){
        
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

        if (point_indices->indices.size() > 100)
        {
            cout << YELLOW << "\n- ITERACIÓN: " << 0 << " -------------------------------" << RESET << endl;
            cout << "Plane_" << this->counter << " found" << endl;
            this->initial_plane.setPlane(plane_coeffs, point_indices, this->cloud_search_xyz);
            
            // arvc::remove_indices_from_cloud(this->cloud_search_xyz, this->initial_plane.inliers);
            
            this->counter++;
        }
    }


    arvc::plane detect_plane(const Eigen::Vector3f& direction){
        
        pcl::SACSegmentation<PointT> ransac;
        arvc::plane plane;

        ransac.setInputCloud(this->cloud_in_xyz);
        ransac.setOptimizeCoefficients(true);
        ransac.setModelType(pcl::SACMODEL_PLANE);
        ransac.setMethodType(pcl::SAC_RANSAC);
        ransac.setMaxIterations(1000);
        ransac.setDistanceThreshold(0.02);
        ransac.setAxis(direction);
        ransac.segment(*plane.inliers, *plane.coeffs);

        if (plane.inliers->indices.size() > 100)
        {
            cout << "Plane_" << this->counter << " found" << endl;
            plane.setPlane(plane.coeffs, plane.inliers, this->cloud_in_xyz);
            // this->add_indices_to_view(plane.inliers, this->cloud_in_xyz); 
            // td.add_plane_to_view(detected_plane);
            // arvc::remove_indices_from_cloud(this->cloud_in_xyz, plane.inliers);

            this->counter++;
        }
        else
        {
            cout << "Plane not found" << endl;
            plane = arvc::plane();
        }
        
        cout << "Plane Inside Detect Plane: " << endl;
        cout << plane << endl;

        return plane;
    }


    arvc::plane detect_perpendicular_plane(const arvc::direction& direction){
        
        pcl::SACSegmentation<PointT> ransac;
        arvc::plane plane;

        // arvc::visualizeCloud(this->cloud_in_xyz);

        ransac.setInputCloud(this->cloud_in_xyz);
        ransac.setOptimizeCoefficients(true);
        ransac.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        ransac.setMethodType(pcl::SAC_RANSAC);
        ransac.setMaxIterations(10000);
        ransac.setDistanceThreshold(0.02);
        ransac.setAxis(direction.vector);
        ransac.setEpsAngle (pcl::deg2rad (5.0));
        ransac.segment(*plane.inliers, *plane.coeffs);

        if (plane.inliers->indices.size() > 300)
        {
            cout << "Plane_" << this->counter << " found" << endl;
            plane.setPlane(plane.coeffs, plane.inliers, this->cloud_in_xyz);
            this->add_indices_to_view(plane.inliers, this->cloud_in_xyz, direction.color); 
            // this->add_plane_to_view(plane);
            arvc::remove_indices_from_cloud(this->cloud_in_xyz, plane.inliers);

            this->counter++;
        }

        else
        {
            cout << "Plane not found" << endl;
            plane = arvc::plane();
        }
        
        // cout << "Plane Inside Detect Plane: " << endl;
        // cout << plane << endl;

        return plane;
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


    void tf_base_to_sensor(Eigen::Vector3f vector){
        this->second_direction = this->RobotBaseToSensorTF.linear() * vector;
    }


    void compute_third_direction(){
        this->third_direction = this->basis.cross(this->second_direction);
    }


    void get_search_directions(){
        arvc::direction direction;
        direction.vector = this->second_direction;
        direction.color = arvc::color(255,0,0);
        this->search_directions[0] = direction;

        direction.vector = this->third_direction;
        direction.color = arvc::color(0,255,0);
        this->search_directions[1] = direction;

        direction.vector = this->basis;
        direction.color = arvc::color(0,0,255);
        this->search_directions[2] = direction;
    }


    void view_result()
    {
        if (fs::exists("camera.txt"))
            this->viewer->loadCameraParameters("camera.txt");

        this->viewer->setBackgroundColor(0.0, 0.0, 0.0);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(this->cloud_in_xyz, 255, 255, 255);
        this->viewer->addPointCloud<pcl::PointXYZ>(this->cloud_in_xyz, color, "cloud_search", this->v1);

        this->add_sensor_origin();
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
        this->viewer->addText3D("os_0", origin_point, 0.02, 0.0, 0.0, 0.0, "origin_text");

        // SENSOR COORDINATE SYSTEM 
        this->viewer->addCoordinateSystem(0.1, this->RobotBaseToSensorTF, "sensor");
        pcl::PointXYZ sensor_origin_point;
        sensor_origin_point.x = this->RobotBaseToSensorTF.translation().x();
        sensor_origin_point.y = this->RobotBaseToSensorTF.translation().y();
        sensor_origin_point.z = this->RobotBaseToSensorTF.translation().z();
        this->viewer->addSphere(sensor_origin_point, 0.02, "sphere2");
        this->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "sphere2");
        this->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "sphere2");
        this->viewer->addText3D("os_sensor", sensor_origin_point, 0.02, 0.0, 0.0, 0.0, "sensor_origin_text");

        /*      
        Eigen::Vector3f vector_x;
        vector_x = this->RobotBaseToSensorTF * Eigen::Vector3f(5,0,0);
        pcl::PointXYZ x_vector(vector_x[0], vector_x[1], vector_x[2]);
        this->viewer->addLine(sensor_origin_point, x_vector, 1.0, 0.0, 0.0, "line_x"); 
        */
       
    }


    void add_normal_to_view(arvc::plane& _plane)
    {
        pcl::PointXYZ centroid;
        pcl::computeCentroid(*_plane.cloud, centroid);

        // CREATE THE NORMAL VECTOR IN EIGEN FORMAT
        Eigen::Translation3f translation(centroid.x, centroid.y, centroid.z);

        // MAKE THE NORMAL VECTOR SMALLER        
        _plane.normal = -_plane.normal / 5;
        _plane.normal = translation * _plane.normal;


        // SAVE THE NORMAL VECTOR IN A PCL FORMAT
        pcl::PointXYZ plane_normal;
        plane_normal.x = _plane.normal[0];
        plane_normal.y = _plane.normal[1];
        plane_normal.z = _plane.normal[2];


        this->viewer->addArrow<pcl::PointXYZ, pcl::PointXYZ>(plane_normal, centroid, 1.0, 1.0, 0.0, false, "normal", this->v2);
        this->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 10, "normal");

    }


    void add_plane_to_view(const arvc::plane& _plane){

        if (_plane.inliers->indices.size() < 100)
            return;

        pcl::PointXYZ centroid;
        pcl::computeCentroid(*this->cloud_search_xyz, _plane.inliers->indices, centroid);

        this->ss.str("");
        this->ss << "plane_" << this->counter;

        this->viewer->addPlane(*_plane.coeffs, centroid.x, centroid.y, centroid.z, this->ss.str(), this->v2);
        this->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, _plane.color.r / 255, _plane.color.g / 255, _plane.color.b / 255, this->ss.str());
        this->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, this->ss.str());
    }


    void add_indices_to_view(pcl::PointIndicesPtr& indices, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_origin, arvc::color color){
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud_origin, *indices, *cloud);

        this->ss.str("");
        this->ss << "cloud_" << this->counter;

        cout << "Adding " << this->ss.str() << " to the viewer with " << cloud->points.size() << " points" << endl;

        // pcl::RGB color;
        // color = arvc::get_random_color(256, 0);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, (int) color.r, (int) color.g, (int) color.b);
        this->viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, this->ss.str(), this->v2);

    }
    
    
    void hold_view()
    {
        while (!this->viewer->wasStopped())
        {
            // this->viewer->saveCameraParameters("camera.txt");
            this->viewer->spinOnce(100);
        }
    }


    void draw_search_directions()
    {
        pcl::PointXYZ origin(0,0,0);

        pcl::PointXYZ x_vector(this->second_direction[0], this->second_direction[1], this->second_direction[2]);
        pcl::PointXYZ y_vector(this->third_direction[0], this->third_direction[1], this->third_direction[2]);
        pcl::PointXYZ z_vector(this->basis[0], this->basis[1], this->basis[2]);

        viewer->addArrow<pcl::PointXYZ, pcl::PointXYZ>(x_vector, origin, 1.0, 0.0, 0.0, false, "x_direction", this->v2);
        viewer->addArrow<pcl::PointXYZ, pcl::PointXYZ>(y_vector, origin, 0.0, 1.0, 0.0, false, "y_direction", this->v2);
        viewer->addArrow<pcl::PointXYZ, pcl::PointXYZ>(z_vector, origin, 0.0, 0.0, 1.0, false, "z_direction", this->v2);

    }


    void add_plane_eigenvectors(const arvc::plane& _plane)
    {
        arvc::axes3d axis3D = arvc::compute_eigenvectors3D(_plane.cloud);
        cout << "Eigenvectors: " << endl;
        cout << axis3D << endl;
        pcl::PointXYZ centroid;
        pcl::computeCentroid(*_plane.cloud, centroid);
        this->viewer->addArrow<pcl::PointXYZ, pcl::PointXYZ>(axis3D.getPoint(axis3D.x, centroid.getVector4fMap()), centroid, 1.0, 0.0, 0.0, false, "eigenvector_1", this->v2);
        this->viewer->addArrow<pcl::PointXYZ, pcl::PointXYZ>(axis3D.getPoint(axis3D.y, centroid.getVector4fMap()), centroid, 0.0, 1.0, 0.0, false, "eigenvector_2", this->v2);
        this->viewer->addArrow<pcl::PointXYZ, pcl::PointXYZ>(axis3D.getPoint(axis3D.z, centroid.getVector4fMap()), centroid, 0.0, 0.0, 1.0, false, "eigenvector_3", this->v2);
        
    }


    private:
        pcl::visualization::PCLVisualizer::Ptr viewer;
        int v1;
        int v2;
    
    public:
    // PUBLIC ATTRIBUTES
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_xyz;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_search_xyz;

    Eigen::Affine3f RobotBaseToSensorTF;
    float range_dist;

    arvc::plane initial_plane;

    Eigen::Vector3f basis;
    Eigen::Vector3f second_direction;
    Eigen::Vector3f third_direction;

    Eigen::Vector3f normal;
    int counter = 0;
    stringstream ss;
    vector<arvc::direction> search_directions;

};


int main(int argc, char const *argv[])
{
    std::cout << setprecision(3) << std::fixed;
    const double RANGE_DIST = 1;

    truss_plane_detector td(RANGE_DIST);

    td.RobotBaseToSensorTF.translation() = Eigen::Vector3f(0.1, 0.0, 0.3);
    td.RobotBaseToSensorTF.linear() = Eigen::Quaternionf::Identity().toRotationMatrix();

    // COMMENT OPTION DEPENDING ON THE COMPUTER YOU ARE USING
    td.cloud_in_xyz = arvc::readCloud("../examples/example_cloud.pcd");
    // td.cloud_in_xyz = arvc::readCloud("/home/arvc/arvc_saved_cloud.pcd");

    // FILTERS TO APPLY TO THE INPUT CLOUD
    td.cloud_in_xyz = arvc::remove_origin_points(td.cloud_in_xyz);
    // td.cloud_in_xyz = arvc::voxel_filter(td.cloud_in_xyz, 0.05);

    // APPLY TRANSFORMATION TO THE ROBOT BASE
    pcl::transformPointCloud(*td.cloud_in_xyz, *td.cloud_in_xyz, td.RobotBaseToSensorTF);
    // pcl::transformPointCloud(*td.cloud_search_xyz, *td.cloud_search_xyz, td.RobotBaseToSensorTF); // No es necesario si se transforma la inicial

    td.get_close_points();
    td.detect_initial_plane();
    td.initial_plane.projectOnPlane();
    // td.add_indices_to_view(td.initial_plane.inliers, td.cloud_search_xyz, arvc::color(255,0,0));
    // td.add_plane_eigenvectors(td.initial_plane);


    // AÑADO NUBE INICIAL
    arvc::viewer view;

    // APLICO LA TRANSFORMACIÓN
    Eigen::Affine3f tf;
    tf.translation() = td.initial_plane.centroid.head<3>();
    tf.linear() = td.initial_plane.eigenvectors.getRotationMatrix();
    pcl::transformPointCloud(*td.initial_plane.cloud, *td.initial_plane.cloud, tf.inverse());

    view.addCloud(td.initial_plane.cloud, arvc::color::GREEN_COLOR);

    pcl::PointXYZ max_point;
    pcl::PointXYZ min_point;

    pcl::getMinMax3D(*td.initial_plane.cloud, min_point, max_point);
    view.addCube(min_point, max_point);

    view.addOrigin();
    // view.addCoordinateSystem(tf);
    // view.addEigenVectors(td.initial_plane.centroid.head<3>(), td.initial_plane.eigenvectors);
    view.show();

    // view.addCoordinateSystem(tf, view.v1);   

    // td.tf_base_to_sensor(td.initial_plane.normal);
    // td.compute_third_direction();
    // td.get_search_directions();

    // int count = 1;
    // int cloud_initial_size = td.cloud_in_xyz->points.size();
    // int detected_plane_size = 1000;

/*     for (const arvc::direction& search_direction : td.search_directions)
    {
        cout << RED << "Search direction: " << search_direction.vector.transpose() << RESET << endl;

        do
        {
            cout << YELLOW << "\n- ITERACIÓN: " << count << " -------------------------------" << RESET << endl;

            arvc::plane detected_plane = td.detect_perpendicular_plane(search_direction);
            arvc::axes3d axis3D = arvc::compute_eigenvectors3D(detected_plane.cloud);
            detected_plane_size = detected_plane.inliers->indices.size();  
            
            count++;
        } while (detected_plane_size > 300 && td.cloud_in_xyz->points.size() > cloud_initial_size * 0.1);
    }
     */


    
    // td.view_result();

    return 0;
}



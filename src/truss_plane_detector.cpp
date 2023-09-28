#include "arvc_utils.hpp"

#include <ignition/math/Matrix4.hh>
#include <ignition/math.hh>
#include <pcl/pcl_config.h>
#include <pcl/common/angles.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Geometry>

/* 
    TODO:
    - Add a function to get the initial plane from the point cloud-> DONE
    - Add a function to get the perpendicular plane from the point cloud-> DONE
    - Set a constraint that the eigenvectors of a plane have to match the search directions. -> PENDING
 */

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
        this->first_direction = Eigen::Vector3f(0,0,1);
        // this->viewer.reset(new pcl::visualization::PCLVisualizer("viewer"));
        // this->v1=(0);
        // this->v2=(1);
        // this->viewer->createViewPort(0.0, 0.0, 0.5, 1.0, this->v1);
        // this->viewer->createViewPort(0.5, 0.0, 1.0, 1.0, this->v2);
        this->search_directions = arvc::axes3d();
    }

    // CONSTRUCTOR
    truss_plane_detector(double _range_dist)
    {
        this->cloud_in_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
        this->cloud_search_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);

        this->range_dist = _range_dist;
        this->RobotBaseToSensorTF = Eigen::Affine3f::Identity();
        this->first_direction = Eigen::Vector3f(0,0,1);
        // this->viewer.reset(new pcl::visualization::PCLVisualizer("viewer"));
        // this->v1=(0);
        // this->v2=(1);
        // this->viewer->createViewPort(0.0, 0.0, 0.5, 1.0, this->v1);
        // this->viewer->createViewPort(0.5, 0.0, 1.0, 1.0, this->v2);
        this->search_directions = arvc::axes3d();

        this->length_threshold = 0;
        this->width_threshold = 0;
        this->initial_plane_found = false;
    }

    // DESTRUCTOR
    ~truss_plane_detector()
    {
        this->cloud_in_xyz.reset();
        this->cloud_search_xyz.reset();
        // this->viewer.reset();
    }


    void detect_initial_plane(){
        
        pcl::PointIndicesPtr point_indices (new pcl::PointIndices);
        pcl::SACSegmentation<PointT> ransac;
        pcl::ModelCoefficientsPtr plane_coeffs (new pcl::ModelCoefficients);


        cout << YELLOW << "\n- DETECTING INITIAL PLANE -------------------------------" << RESET << endl;
        int intento = 0;
        bool all_clusters_valid = false;
        const int initial_size = this->cloud_search_xyz->points.size();

        do
        {
            cout << "\n- ITERACIÓN: " << intento << " -------------------------------" << endl;

            // TERMINATE FUNCTION IF THE CLOUD IS TOO SMALL
            if (this->cloud_search_xyz->points.size() < initial_size * 0.01)
            {
                cout << RED << "So small cloud to find an initial plane" << RESET << endl;
                return;
            }

            // BIGGEST PLANE EXTRACTION WITH RANSAC
            ransac.setInputCloud(this->cloud_search_xyz);
            ransac.setOptimizeCoefficients(true);
            ransac.setModelType(pcl::SACMODEL_PLANE);
            ransac.setMethodType(pcl::SAC_RANSAC);
            ransac.setMaxIterations(1000);
            ransac.setDistanceThreshold(0.02);
            ransac.segment(*point_indices, *plane_coeffs);
            
            // SETUP THE INITIAL PLANE OBJECT
            this->initial_plane.setPlane(plane_coeffs, point_indices, this->cloud_search_xyz);

            
            // VALIDATE PLANES FOUND

            if(!this->validate_plane(this->initial_plane))
            {
                all_clusters_valid = false;
                vector<arvc::plane> plane_clusters;

                // DIVIDE UN MULTIPLE CLUSTERS
                plane_clusters = this->get_euclidean_clusters(this->initial_plane);
                cout << "Initial plane cloud divided in " << plane_clusters.size() << " clusters" << endl;

                if( plane_clusters.size() == 0) // if no clusters found, try to validate the complete initial plane
                { 
                    cout << YELLOW << "WARNING: No clusters found." << RESET << endl;
                    all_clusters_valid = false;
                }

                if (plane_clusters.size() == 1)
                    all_clusters_valid = false;

                else if (plane_clusters.size() > 1)
                {
                    arvc::viewer tmp_viewer;
                    // EVERY CLUSTER MUST BE VALID
                    int clust_count = 0;
                    for(arvc::plane plano : plane_clusters){
                        
                        tmp_viewer.addCloud(plano.cloud, plano.color);

                        if(this->validate_plane(plano))
                        {
                            all_clusters_valid = true;
                            cout << "Cluster " << clust_count << " valido" << endl;
                        }
                        else
                        {
                            all_clusters_valid = false;
                            cout << "Cluster " << clust_count << " invalido" << endl;
                            // break;
                        }
                        clust_count++;
                    }
                    tmp_viewer.show( );
                }
            }

            // GET OUT THE LOOP IF ALL CLUSTERS ARE VALID
            if (all_clusters_valid){
                this->initial_plane_found = true;
                cout << GREEN << "+ Initial plane found." << RESET << endl;
                return;
            }
            else
                cout << YELLOW << "- Initial plane not valid. Trying in next iteration." << RESET << endl;

            // REMOVE THE INLIERS FROM THE CLOUD UNTIL THE SIZE IS LESS THAN 10% OF THE INITIAL SIZE
            arvc::remove_indices_from_cloud(this->cloud_search_xyz, this->initial_plane.inliers);

            intento++;

        } while (!all_clusters_valid);
        
        this->counter++;
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


    arvc::plane detect_perpendicular_plane(const Eigen::Vector3f& _direction){
        
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(this->cloud_in_xyz);

        pcl::SACSegmentation<PointT> ransac;
        static arvc::plane plane;
        vector<arvc::plane> plane_clusters;

        ransac.setInputCloud(this->cloud_in_xyz);
        ransac.setOptimizeCoefficients(true);
        ransac.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        ransac.setMethodType(pcl::SAC_RANSAC);
        ransac.setMaxIterations(1000);
        ransac.setSamplesMaxDist(0.5, tree); //TODO
        ransac.setDistanceThreshold(0.02);
        ransac.setAxis(_direction.normalized());
        ransac.setEpsAngle (pcl::deg2rad (5.0));

        ransac.segment(*plane.inliers, *plane.coeffs);
        plane.setPlane(plane.coeffs, plane.inliers, this->cloud_in_xyz);

        cout << GREEN << "\n----------------------------------------" << endl;
        cout <<"Direction Plane Detected: " << endl;
        cout << plane << RESET << endl << endl;

        if(!this->validate_plane(plane))
        {
            // DIVIDE UN MULTIPLE CLUSTERS
            plane_clusters = this->get_euclidean_clusters(plane);
            cout << "Clusters Detectados por Dirección: " << plane_clusters.size() << endl;

            if( plane_clusters.size() == 0) // if no clusters found, try to validate the complete initial plane
            { 
                cout << YELLOW << "WARNING: No clusters found." << RESET << endl;
            }

            else if (plane_clusters.size() > 0)
            {
                
                for(arvc::plane plano : plane_clusters){

                    cout << "Plano length: " << plano.length << endl;
                    cout << "Plano width: " << plano.width << endl;

                    if(this->validate_plane(plano)){
                        this->detected_planes.push_back(plano);
                        this->counter++;
                    }
                    else
                        cout << "Plano invalido detectado" << endl;
                }
            }
        }

        arvc::remove_indices_from_cloud(this->cloud_in_xyz, plane.inliers);

        return plane;
    }


/*   arvc::plane detect_perpendicular_plane(const arvc::axes3d& _search_directions){
        
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(this->cloud_in_xyz);

        pcl::SACSegmentation<PointT> ransac;
        arvc::plane plane;

        ransac.setInputCloud(this->cloud_in_xyz);
        ransac.setOptimizeCoefficients(true);
        ransac.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        ransac.setMethodType(pcl::SAC_RANSAC);
        ransac.setMaxIterations(10000);
        ransac.setSamplesMaxDist(0.5, tree); //TODO
        ransac.setDistanceThreshold(0.02);
        ransac.setAxis(direction.vector);
        ransac.setEpsAngle (pcl::deg2rad (5.0));
        ransac.segment(*plane.inliers, *plane.coeffs);

        if (plane.inliers->indices.size() > 300)
        {
            cout << "Plane_" << this->counter << " found" << endl;
            plane.setPlane(plane.coeffs, plane.inliers, this->cloud_in_xyz);
            // this->add_indices_to_view(plane.inliers, this->cloud_in_xyz, direction.color); 
            arvc::remove_indices_from_cloud(this->cloud_in_xyz, plane.inliers);

            this->counter++;
        }

        else
        {
            cout << "Plane not found" << endl;
            plane = arvc::plane();
        }
        
        return plane;
    } */


    void get_close_points(){
        
        pcl::IndicesPtr indices(new pcl::Indices);
        const float range = this->range_dist;

        Eigen::Vector4f min_pt(-range_dist, -range_dist, -range_dist, 1.0);
        Eigen::Vector4f max_pt(range_dist, range_dist, range_dist, 1.0);

        pcl::getPointsInBox(*this->cloud_in_xyz, min_pt, max_pt, *indices);
        
        pcl::ExtractIndices<PointT> extract;
        
        extract.setInputCloud(this->cloud_in_xyz);
        extract.setIndices(indices);
        extract.setNegative(false);
        extract.filter(*this->cloud_search_xyz);

    }


    void tf_base_to_sensor(Eigen::Vector3f vector){
        this->second_direction = this->RobotBaseToSensorTF.linear() * vector;
    }


    void compute_second_direction(){
        this->second_direction.x() = this->initial_plane.coeffs->values[0];
        this->second_direction.y() = this->initial_plane.coeffs->values[1];
        this->second_direction.z() = this->initial_plane.coeffs->values[2];
    }


    void compute_third_direction(){
        this->third_direction = this->first_direction.cross(this->second_direction);
    }


    void get_search_directions(){
        this->search_directions.x = this->second_direction;
        this->search_directions.y = this->third_direction;
        this->search_directions.z = this->first_direction;

        // direction.vector = this->second_direction;
        // direction.color = arvc::color(255,0,0);
        // this->search_directions[0] = direction;

        // direction.vector = this->third_direction;
        // direction.color = arvc::color(0,255,0);
        // this->search_directions[1] = direction;

        // direction.vector = this->first_direction;
        // direction.color = arvc::color(0,0,255);
        // this->search_directions[2] = direction;
    }


/*     void view_result()
    {
        if (fs::exists("camera.txt"))
            this->viewer->loadCameraParameters("camera.txt");

        this->viewer->setBackgroundColor(0.0, 0.0, 0.0);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(this->cloud_in_xyz, 255, 255, 255);
        this->viewer->addPointCloud<pcl::PointXYZ>(this->cloud_in_xyz, color, "cloud_search", this->v1);

        this->add_sensor_origin();
        this->draw_search_directions();

        this->hold_view();
 
    } */


/*     void add_sensor_origin()
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
       
    }
 */


/*     void add_normal_to_view(arvc::plane& _plane)
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

    } */


/*     void add_plane_to_view(const arvc::plane& _plane){

        if (_plane.inliers->indices.size() < 100)
            return;

        pcl::PointXYZ centroid;
        pcl::computeCentroid(*this->cloud_search_xyz, _plane.inliers->indices, centroid);

        this->ss.str("");
        this->ss << "plane_" << this->counter;

        this->viewer->addPlane(*_plane.coeffs, centroid.x, centroid.y, centroid.z, this->ss.str(), this->v2);
        this->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, _plane.color.r / 255, _plane.color.g / 255, _plane.color.b / 255, this->ss.str());
        this->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, this->ss.str());
    } */


/*     void add_indices_to_view(pcl::PointIndicesPtr& indices, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_origin, arvc::color color){
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud_origin, *indices, *cloud);

        this->ss.str("");
        this->ss << "cloud_" << this->counter;

        cout << "Adding " << this->ss.str() << " to the viewer with " << cloud->points.size() << " points" << endl;

        // pcl::RGB color;
        // color = arvc::get_random_color(256, 0);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, (int) color.r, (int) color.g, (int) color.b);
        this->viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, this->ss.str(), this->v2);

    } */
    
    
/*     void hold_view()
    {
        while (!this->viewer->wasStopped())
        {
            // this->viewer->saveCameraParameters("camera.txt");
            this->viewer->spinOnce(100);
        }
    } */


    void draw_search_directions(const pcl::visualization::PCLVisualizer::Ptr& _viewer, const int& _viewport=0)
    {
        pcl::PointXYZ origin(0,0,0);

        pcl::PointXYZ x_vector(this->second_direction[0], this->second_direction[1], this->second_direction[2]);
        pcl::PointXYZ y_vector(this->third_direction[0], this->third_direction[1], this->third_direction[2]);
        pcl::PointXYZ z_vector(this->first_direction[0], this->first_direction[1], this->first_direction[2]);

        _viewer->addArrow<pcl::PointXYZ, pcl::PointXYZ>(x_vector, origin, 1.0, 0.0, 0.0, false, "x_direction", _viewport);
        _viewer->addArrow<pcl::PointXYZ, pcl::PointXYZ>(y_vector, origin, 0.0, 1.0, 0.0, false, "y_direction", _viewport);
        _viewer->addArrow<pcl::PointXYZ, pcl::PointXYZ>(z_vector, origin, 0.0, 0.0, 1.0, false, "z_direction", _viewport);

    }


/*     void add_plane_eigenvectors(const arvc::plane& _plane)
    {
        arvc::axes3d axis3D = arvc::compute_eigenvectors3D(_plane.cloud);

        pcl::PointXYZ centroid;
        pcl::computeCentroid(*_plane.cloud, centroid);
        this->viewer->addArrow<pcl::PointXYZ, pcl::PointXYZ>(axis3D.getPoint(axis3D.x, centroid.getVector4fMap()), centroid, 1.0, 0.0, 0.0, false, "eigenvector_1", this->v2);
        this->viewer->addArrow<pcl::PointXYZ, pcl::PointXYZ>(axis3D.getPoint(axis3D.y, centroid.getVector4fMap()), centroid, 0.0, 1.0, 0.0, false, "eigenvector_2", this->v2);
        this->viewer->addArrow<pcl::PointXYZ, pcl::PointXYZ>(axis3D.getPoint(axis3D.z, centroid.getVector4fMap()), centroid, 0.0, 0.0, 1.0, false, "eigenvector_3", this->v2);
        
    } */


    bool validate_plane(const arvc::plane& _plane){

        // As it has been established that all clusters must be valid, this statement invalidates many valid clusters.
        /* if (_plane.inliers->indices.size() < 200)
            return false; */

        if (_plane.length < this->length_threshold
         && _plane.width < this->width_threshold
         && _plane.length > 0.05
         && _plane.width > 0.05)
            return true;
        else
            return false;
    }


    vector<arvc::plane> get_euclidean_clusters(const arvc::plane& _plane){

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (_plane.cloud);
        std::vector<pcl::PointIndices> cluster_indices;

        vector<arvc::plane> plane_clusters;

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.025); // 2cm
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (_plane.cloud->points.size()+1);
        ec.setSearchMethod (tree);
        ec.setInputCloud (_plane.cloud);
        ec.extract(cluster_indices);

        cout << "CLUSTERS DETECTADOS: " << cluster_indices.size() << endl;
        if (cluster_indices.size() > 0)
        {
            // arvc::viewer tmp_viewer;
            // tmp_viewer.addCloud(_plane.cloud);

            for (pcl::PointIndices cluster : cluster_indices)
            {
                arvc::plane tmp_plane;
                pcl::PointIndicesPtr indices_ptr(new pcl::PointIndices);

                *indices_ptr = cluster;

                if (!this->initial_plane_found)
                {   cout << "Aun no se ha encontrado el plano inicial" << endl;
                    tmp_plane.setPlane(_plane.coeffs, indices_ptr, _plane.cloud);
                }
                else
                {
                    cout << "Se establece forzando las direcciones de los autovectores" << endl;
                    tmp_plane.setPlane(_plane.coeffs, indices_ptr, _plane.cloud, this->search_directions);
                }
                plane_clusters.push_back(tmp_plane);
                // tmp_viewer.addCloud(tmp_plane.cloud, tmp_plane.color);
            }
            // tmp_viewer.show();
        }
        else
            plane_clusters.resize(0);

    
        return plane_clusters;
    }


    private:
        // pcl::visualization::PCLVisualizer::Ptr viewer;
        // int v1;
        // int v2;
    
    public:
    // PUBLIC ATTRIBUTES
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_xyz;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_search_xyz;

    Eigen::Affine3f RobotBaseToSensorTF;
    float range_dist;

    arvc::plane initial_plane;
    bool initial_plane_found;

    vector<arvc::plane> detected_planes;

    Eigen::Vector3f first_direction;
    Eigen::Vector3f second_direction;
    Eigen::Vector3f third_direction;

    Eigen::Vector3f normal;
    int counter = 0;
    stringstream ss;
    arvc::axes3d search_directions;
    float length_threshold;
    float width_threshold;

};


int main(int argc, char const *argv[])
{
    std::cout << setprecision(3) << std::fixed;
    const double RANGE_DIST = 1;

    truss_plane_detector td(RANGE_DIST);

    td.length_threshold = 1.0;
    td.width_threshold = 0.25;

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
    td.compute_second_direction();
    td.compute_third_direction();
    td.get_search_directions();

    // AÑADO NUBE INICIAL

    arvc::viewer view;
    view.addCloud(td.cloud_in_xyz);
    // td.draw_search_directions(view.view);

    // for (const arvc::plane& _plane : td.init_plane_clusters)
    // {
    //     view.addCloud(_plane.cloud, _plane.color);
    //     view.addEigenVectors(_plane.centroid.head<3>(), _plane.eigenvectors);
    //     view.addPolygon(_plane.polygon, _plane.color);
    // }
    

    td.detect_perpendicular_plane(td.search_directions.x);
    // td.detect_perpendicular_plane(td.search_directions.y);
    // td.detect_perpendicular_plane(td.search_directions.z);

    cout << "\n--------------------------------------------------" << endl;
    cout << "Num of Detected planes: " << td.detected_planes.size() << endl;

    for (const arvc::plane& _plane : td.detected_planes)
    {
        // cout << "TMP PLANE: " << endl;
        // cout << _plane << endl;
        view.addCloud(_plane.cloud, _plane.color);
        view.addEigenVectors(_plane.centroid.head<3>(), _plane.eigenvectors);
        view.addPolygon(_plane.polygon, _plane.color);
        // view.addPlane(_plane, _plane.color);
    }

    view.addOrigin();
    view.show();

/*    view.addCoordinateSystem(tf, view.v1);   

    td.tf_base_to_sensor(td.initial_plane.normal);
    td.compute_third_direction();
    td.get_search_directions();

    int count = 1;
    int cloud_initial_size = td.cloud_in_xyz->points.size();
    int detected_plane_size = 1000;

     for (const arvc::direction& search_direction : td.search_directions)
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


    
    return 0;
}



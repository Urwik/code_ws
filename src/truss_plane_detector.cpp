#include "arvc_utils.hpp"

#include <ignition/math/Matrix4.hh>
#include <ignition/math.hh>
#include <pcl/pcl_config.h>
#include <pcl/common/angles.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/filters/passthrough.h>


#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <Eigen/Geometry>


namespace im = ignition::math;

arvc::console cons;


bool compare_plane_model_d(pcl::ModelCoefficients _model_1,pcl::ModelCoefficients _model_2){
    return (_model_1.values[3] < _model_2.values[3]);
}

class truss_plane_detector
{   

public:

    // DEFAULT CONSTRUCTOR
    truss_plane_detector()
    {
        this->cloud_in_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
        this->cloud_search_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
        this->remain_indices.reset(new pcl::PointIndices);
        this->intersection_indices.reset(new pcl::PointIndices);
        this->cloud_intersection_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
        
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
        this->remain_indices.reset(new pcl::PointIndices);
        this->intersection_indices.reset(new pcl::PointIndices);
        this->cloud_intersection_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
        this->tree.reset(new pcl::search::KdTree<pcl::PointXYZ>);

        

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


    /**
     * @brief Detect a plane on the truss to set the search directions
    */
    void detect_initial_plane(){
        
        pcl::PointIndicesPtr point_indices (new pcl::PointIndices);
        pcl::SACSegmentation<PointT> ransac;
        pcl::ModelCoefficientsPtr plane_coeffs (new pcl::ModelCoefficients);

        // CREATE A VECTOR WITH ALL THE INDICES OF THE CLOUD
        this->remain_indices->indices.resize(this->cloud_in_xyz->points.size());
        for (int i = 0; i < this->cloud_in_xyz->points.size(); i++)
            this->remain_indices->indices[i] = i;
        
        pcl::copyPointCloud(*this->cloud_in_xyz, *this->cloud_search_xyz);

        this->tree->setInputCloud(this->cloud_in_xyz);

        cout << BLUE << "\n- DETECTING INITIAL PLANE -------------------------------" << RESET << endl;
        int intento = 0;
        bool all_clusters_valid = false;
        const int initial_size = this->cloud_search_xyz->points.size();

        do
        {
            cout << "\n- ITERACIÓN: " << intento << " -------------------------------" << endl;

            // TERMINATE FUNCTION IF THE CLOUD IS TOO SMALL
            if (this->cloud_search_xyz->points.size() < initial_size * 0.01) {
                cout << RED << "Cloud to small to find an initial plane" << RESET << endl; return;
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
                cout << " - Initial plane divided in " << plane_clusters.size() << " clusters" << endl;

                if( plane_clusters.size() == 0) // if no clusters found, try to validate the complete initial plane
                { 
                    cout << RED << "WARNING: No clusters found." << RESET << endl;
                    all_clusters_valid = false;
                }

                else if (plane_clusters.size() == 1)
                    all_clusters_valid = false;

                else if (plane_clusters.size() > 1)
                {
                    // EVERY CLUSTER MUST BE VALID
                    int clust_count = 0;
                    for(arvc::plane plano : plane_clusters){
                        if(this->validate_plane(plano))
                        {
                            all_clusters_valid = true;
                            // cout << "Cluster " << clust_count << " valido" << endl;
                        }
                        else
                        {
                            all_clusters_valid = false;
                            // cout << "Cluster " << clust_count << " invalido" << endl;
                        }
                        clust_count++;
                    }
                }
            }
            else
                all_clusters_valid = true;


            // GET OUT THE LOOP IF ALL CLUSTERS ARE VALID
            if (all_clusters_valid){
                this->initial_plane_found = true;
                cout << GREEN << " +++ Initial plane found. Setting search directions +++++" << RESET << endl;
                cout << BLUE <<  " --------------------------------------------------------" << RESET << endl;
                return;
            }
            else{
                cout << YELLOW << " --- Initial plane not valid. Trying in next iteration." << RESET << endl;
            }

            // REMOVE THE INLIERS FROM THE CLOUD UNTIL THE SIZE IS LESS THAN 10% OF THE INITIAL SIZE
            arvc::remove_indices_from_cloud(this->cloud_search_xyz, this->initial_plane.inliers);

            intento++;

        } while (!all_clusters_valid);
        
        this->counter++;
    }


    /**
     * @brief Detect a plane perpendicular to the desired direction
     * @param _direction: Desired search direction
     * @return arvc::plane: Plane object
    */
    arvc::plane detect_perpendicular_plane(const Eigen::Vector3f& _direction){
        
        pcl::SACSegmentation<PointT> ransac;
        arvc::plane plane;
        vector<arvc::plane> plane_clusters;

        ransac.setInputCloud(this->cloud_in_xyz);
        ransac.setIndices(this->remain_indices);
        ransac.setOptimizeCoefficients(true);
        ransac.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        ransac.setMethodType(pcl::SAC_RANSAC);
        ransac.setMaxIterations(1000);
        ransac.setSamplesMaxDist(0.5, this->tree); //TODO
        ransac.setDistanceThreshold(0.02);
        ransac.setAxis(_direction.normalized());
        ransac.setEpsAngle (pcl::deg2rad (5.0));

        ransac.segment(*plane.inliers, *plane.coeffs);

        if(plane.inliers->indices.size() > 0)
            plane.setPlane(plane.coeffs, plane.inliers, this->cloud_in_xyz);
        else
        {
            cout << "RANSAC can not extract plane" << endl;
            return plane;
        }

        if(!this->validate_plane(plane))
        {
            // DIVIDE UN MULTIPLE CLUSTERS
            plane_clusters = this->get_euclidean_clusters(plane);
            // cout << "Clusters Detectados por Dirección: " << plane_clusters.size() << endl;

            if( plane_clusters.size() == 0) // if no clusters found, try to validate the complete initial plane
            { 
                cout << YELLOW << "WARNING: No clusters found." << RESET << endl;
            }

            else if (plane_clusters.size() > 0)
            {
                
                for(arvc::plane plano : plane_clusters){
                    if(this->validate_plane(plano)){
                        this->detected_planes.push_back(plano);
                        this->counter++;
                    }
                }
            }
        }

        // this->remove_used_indices(plane.inliers);
        return plane;
    }


    /**
     * @brief Look for planes in a desired direction recursively until no more planes are found
     * @param _direction: Desired search direction
    */
    void search_in_direction(const Eigen::Vector3f& _direction){
        arvc::plane plane;
        int count=0;
        arvc::viewer view2("TMP VIEWER");

        while (true)
        {
            cout << "Plane " << count << " found" << endl;
            view2.addCloud(plane.cloud, plane.color);
            plane = this->detect_perpendicular_plane(_direction);

            if (plane.inliers->indices.size() <= 1000){
                break;
            }
            
            count++;
        }

        view2.show();
    }


    /**
     * @brief Remove a set of indices from the remaining indices not used
     * @param _indices: Indices to remove
    */
    void remove_used_indices(const pcl::PointIndicesPtr& _indices){

        auto new_end = std::remove_if(this->remain_indices->indices.begin(), this->remain_indices->indices.end(), [&](int value) {
        return std::find(_indices->indices.begin(), _indices->indices.end(), value) != _indices->indices.end();});

        this->remain_indices->indices.erase(new_end, this->remain_indices->indices.end());
    }


    /**
     * @brief Crop the point cloud in a box of a certain size
    */
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


    /**
     * @brief Apply transform from base to sensor
    */
    void tf_base_to_sensor(Eigen::Vector3f vector){
        this->second_direction = this->RobotBaseToSensorTF.linear() * vector;
    }


    /**
     * @brief Compute the second search direction
    */
    void compute_second_direction(){
        this->second_direction.x() = this->initial_plane.coeffs->values[0];
        this->second_direction.y() = this->initial_plane.coeffs->values[1];
        this->second_direction.z() = this->initial_plane.coeffs->values[2];
    }


    /**
     * @brief Compute the third search direction
    */
    void compute_third_direction(){
        this->third_direction = this->first_direction.cross(this->second_direction);
    }


    /**
     * @brief Save search directions in an axis3d object
    */
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


    /**
     * @brief Remove the detected planes from the initial cloud
    */
    void remove_detected_planes(){

        pcl::PointIndicesPtr cat_indices(new pcl::PointIndices);

        for (arvc::plane plane : this->detected_planes){
            cat_indices->indices.insert(cat_indices->indices.end(), plane.inliers->indices.begin(), plane.inliers->indices.end());
            
            cout<< "Plane " << this->counter << " with " << plane.inliers->indices.size() << " points removed" << endl;
        }
        
        arvc::remove_indices_from_cloud(this->cloud_in_xyz, cat_indices);
    }


    /**
     * @brief Draw search directions as axis in the viewer
     * @param _viewer: Pointer to a viewer to draw the axis
     * @param _viewport: Viewport to draw the axis
    */
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


    /**
     * @brief Check if a plane is valid as a truss plane
     * @param _plane: Plane to validate
    */
    bool validate_plane(const arvc::plane& _plane){

        // As it has been established that all clusters must be valid, this statement invalidates many valid clusters.
        /* if (_plane.inliers->indices.size() < 200)
            return false; */

        if (_plane.length < this->length_threshold
         && _plane.width < this->width_threshold
         && _plane.length > min_length
         && _plane.width > min_width)
         {
            return true;
         }
        else
        {
            // cout << RED << " + Plane invalid" << YELLOW << endl;
            return false;
        }
    }


    /**
     * @brief Split the @param _plane in clusters using euclidean clustering
     * @param _plane: Plane to split
     * @return vector<arvc::plane>: Vector of planes
    */
    vector<arvc::plane> 
    get_euclidean_clusters(arvc::plane& _plane){

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (_plane.original_cloud);

        std::vector<pcl::PointIndices> cluster_indices;

        vector<arvc::plane> plane_clusters;

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.025); // 2cm
        ec.setMinClusterSize (100);
        _plane.getCloud();
        ec.setMaxClusterSize (_plane.cloud->points.size()+1);
        ec.setSearchMethod (tree);
        ec.setInputCloud (_plane.original_cloud);
        ec.setIndices(_plane.inliers);
        ec.extract(cluster_indices);

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
                    tmp_plane.setPlane(_plane.coeffs, indices_ptr, _plane.original_cloud);
                else
                    tmp_plane.setPlane(_plane.coeffs, indices_ptr, _plane.original_cloud, this->search_directions);

                plane_clusters.push_back(tmp_plane);
                // tmp_viewer.addCloud(tmp_plane.cloud, tmp_plane.color);
            }
            // tmp_viewer.show();
        }
        else
            plane_clusters.resize(0);

        return plane_clusters;
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


/*     void add_plane_eigenvectors(const arvc::plane& _plane)
    {
        arvc::axes3d axis3D = arvc::compute_eigenvectors3D(_plane.cloud);

        pcl::PointXYZ centroid;
        pcl::computeCentroid(*_plane.cloud, centroid);
        this->viewer->addArrow<pcl::PointXYZ, pcl::PointXYZ>(axis3D.getPoint(axis3D.x, centroid.getVector4fMap()), centroid, 1.0, 0.0, 0.0, false, "eigenvector_1", this->v2);
        this->viewer->addArrow<pcl::PointXYZ, pcl::PointXYZ>(axis3D.getPoint(axis3D.y, centroid.getVector4fMap()), centroid, 0.0, 1.0, 0.0, false, "eigenvector_2", this->v2);
        this->viewer->addArrow<pcl::PointXYZ, pcl::PointXYZ>(axis3D.getPoint(axis3D.z, centroid.getVector4fMap()), centroid, 0.0, 0.0, 1.0, false, "eigenvector_3", this->v2);
        
    } */


    // -----------------------------------------------------
    //  NEW SEGMENTATION FUNCTIONS
    // -----------------------------------------------------

    void remove_plane_intersection_indices(){
            
        pcl::PointIndicesPtr cat_indices(new pcl::PointIndices);

        cout << "Cat plane indices" << endl;
        for (arvc::plane plane : this->direction_planes)
            cat_indices->indices.insert(cat_indices->indices.end(), plane.inliers->indices.begin(), plane.inliers->indices.end());

        pcl::PointIndicesPtr intersection_indices(new pcl::PointIndices);
        intersection_indices->indices = arvc::get_duplicates(cat_indices->indices);

        cout << "Indices a eliminar: " << intersection_indices->indices.size() << endl;

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(this->cloud_in_xyz);
        extract.setIndices(intersection_indices);
        extract.setNegative(true);
        // extract.filter(this->remain_indices->indices);
        extract.filter(*this->cloud_in_xyz);
    }


    void get_all_direction_planes(){
        cout << YELLOW << "\n---------------------------------------------------------" << RESET << endl;
        cout << YELLOW << "- LOOKING FOR PLANES IN SEARCH DIRECTIONS -----------------" << RESET << endl;
        
        // GET THE PLANES IN THE THREE DIRECTIONS
        cout << RED << "\n- DIRECTION X -------------------------------" << RESET << endl;
        this->get_planes_in_direction(this->search_directions.x, arvc::color::RED_COLOR);

        cout << GREEN << "\n- DIRECTION Y -------------------------------" << RESET << endl;
        this->get_planes_in_direction(this->search_directions.y, arvc::color::GREEN_COLOR);
       
        cout << BLUE << "\n- DIRECTION Z -------------------------------" << RESET << endl;
        this->get_planes_in_direction(this->search_directions.z, arvc::color::BLUE_COLOR);
        cout << "\nGET ALL DIRECTION PLANES " << GREEN << "DONE" << RESET << endl;
    }


    vector<pcl::ModelCoefficients> check_normal_direction(vector<pcl::ModelCoefficients> _plane_models, vector<pcl::PointIndices> _plane_indices){
        
        cons.debug("# Checking normal direction...");
        float first_normal_sense;
        float tmp_normal_sense;
        bool flip_normal;

        vector<pcl::ModelCoefficients> _corrected_plane_models;
        _corrected_plane_models = _plane_models;

        Eigen::Vector3f normal;
        Eigen::Vector3f origin(0,0,0);
        Eigen::Vector4f centroid;
        PointCloud::Ptr tmp_cloud(new PointCloud);

        for (size_t i = 0; i < _corrected_plane_models.size(); i++)
        {
            flip_normal = false;

            normal.x() = _corrected_plane_models[i].values[0];
            normal.y() = _corrected_plane_models[i].values[1];
            normal.z() = _corrected_plane_models[i].values[2];

            pcl::compute3DCentroid(*this->cloud_in_xyz, _plane_indices[i], centroid);

            if (i == 0){
                first_normal_sense = normal.dot(origin - centroid.head(3));
                cons.error("FIRST NORMAL SENSE: " + to_string(first_normal_sense));
            }
            else{

                tmp_normal_sense = normal.dot(origin - centroid.head(3));
                cons.error("TMP NORMAL SENSE: " + to_string(tmp_normal_sense));

                if (first_normal_sense > 0 && tmp_normal_sense < 0){
                    cons.debug(" - Flipping normal");
                    _corrected_plane_models[i].values[3] = -_corrected_plane_models[i].values[3];
                }
                else if (first_normal_sense < 0 && tmp_normal_sense > 0){
                    cons.debug(" - Flipping normal");
                    _corrected_plane_models[i].values[3] = -_corrected_plane_models[i].values[3];
                }
            }
        }

        return _corrected_plane_models;
    }


    void get_planes_in_direction(const Eigen::Vector3f& _direction, const arvc::color& _color){
        /* 
            TODO
            Una vez detectado un plano ha de eliminarse de la nube de puntos.
            Posteriormente se extraeran los inliers de cada plano y se buscarán aquellos comunes (intersecciones).
        */

        pcl::SACSegmentation<PointT> ransac;
        arvc::plane plane;
        vector<arvc::plane> plane_clusters;
        vector<pcl::ModelCoefficients> _refined_plane_models;

        vector<pcl::ModelCoefficients> _tmp_plane_models;
        vector<pcl::PointIndices> _tmp_plane_indices;

        pcl::ModelCoefficientsPtr _coeffs(new pcl::ModelCoefficients);
        pcl::PointIndicesPtr _inliers(new pcl::PointIndices);

        ransac.setOptimizeCoefficients(true);
        ransac.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        ransac.setMethodType(pcl::SAC_RANSAC);
        ransac.setMaxIterations(5000);
        ransac.setDistanceThreshold(0.025);
        // ransac.setAxis(_direction.normalized());
        ransac.setAxis(_direction);
        ransac.setEpsAngle (pcl::deg2rad (5.0));

        arvc::viewer view("TMP VIEWER_1");

        view.addEigenVectors(Eigen::Vector3f(0,0,0), this->search_directions);

        // LOOP TO FIND ALL PLANES IN THE DIRECTION UNTIL NO MORE PLANES ARE FOUND OR THE CLOUD IS TOO SMALL
        int _count = 0;
        while(this->cloud_search_xyz->points.size() > this->cloud_in_xyz->points.size() * 0.1){

            // CLEAR THE _inliers AND _coeffs VALUES
            _inliers->indices.clear();
            _coeffs->values.clear();

            // COMPUTE PLANE PARAMETERS
            ransac.setInputCloud(this->cloud_search_xyz);
            tree->setInputCloud(this->cloud_search_xyz);
            ransac.setSamplesMaxDist(0.1, this->tree); //TODO
            ransac.segment(*_inliers, *_coeffs);

            // PASS TO NEXT ITERATION IF THE PLANE IS TOO SMALL
            if(_inliers->indices.size() < this->min_plane_inliers)
                break;

            // SHOW SOME INFORMATION
            cout << "Plane_" << _count << ": " ;
            // cout << " - Inliers: " << _inliers->indices.size();
            cout << " - Coeffs: ";
            arvc::print_vector(_coeffs->values);

            // arvc::plane _tmp_plane;
            // _tmp_plane.setPlane(_coeffs, _inliers, this->cloud_search_xyz);

            // arvc::viewer view2("TMP VIEWER");
            // view2.addCloud(_tmp_plane.cloud, _color);
            // view2.addPlane(_tmp_plane, _color);
            // view2.addOrigin();
            // view2.show();


            // SAVE PLANE PARAMETERS
            _tmp_plane_models.push_back(*_coeffs);
            _tmp_plane_indices.push_back(*_inliers);
            // this->plane_inliers.push_back(*_inliers);

            // REMOVE FROM THE SEARCH CLOUD THE ACTUAL PLANE INLIERS
            PointCloud::Ptr tmp_cloud(new PointCloud);
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(this->cloud_search_xyz);
            extract.setIndices(_inliers);
            
            extract.setNegative(false);
            extract.filter(*tmp_cloud);

            extract.setNegative(true);
            extract.filter(*this->cloud_search_xyz);

            view.addCloud(tmp_cloud, _color);

            _count++;
        }

        view.show();
        _tmp_plane_models = this->check_normal_direction(_tmp_plane_models, _tmp_plane_indices);
        _refined_plane_models = this->refine_plane_models(_tmp_plane_models, _direction);

        for (pcl::ModelCoefficients _coeffs : _refined_plane_models)
           this->plane_models.push_back(_coeffs);
    }




    vector<pcl::ModelCoefficients> sort_plane_models(vector<pcl::ModelCoefficients> _plane_models){
        
        cons.debug(" - Sorting plane models...");

        sort(_plane_models.begin(), _plane_models.end(), compare_plane_model_d);
        return _plane_models;
    }


    vector<pcl::ModelCoefficients> refine_plane_models(const vector<pcl::ModelCoefficients>& _plane_models, const Eigen::Vector3f& _direction){
        
        cons.debug(" - Refining plane models...");
        cons.debug(" - Initial plane models: " + to_string(_plane_models.size()));

        vector<pcl::ModelCoefficients> _new_plane_models;
        vector<pcl::ModelCoefficients> _tmp_plane_models; 
        vector<int>::allocator_type _refined_indices;

        pcl::ModelCoefficientsPtr tmp_coeffs (new pcl::ModelCoefficients);
        pcl::PointIndicesPtr tmp_indices (new pcl::PointIndices);

        
        _tmp_plane_models = _plane_models;

        _tmp_plane_models = this->sort_plane_models(_tmp_plane_models);

        for (size_t i = 0; i < _tmp_plane_models.size(); i++)
        {
            cout << "Plane_" << i << ": " ;
            cout << " - Coeffs: ";
            arvc::print_vector(_tmp_plane_models[i].values);

            arvc::viewer view("TMP VIEWER");
            arvc::plane _tmp_plane;
            pcl::PointIndicesPtr _tmp_inliers(new pcl::PointIndices);


            view.addCloud(_tmp_plane.cloud, arvc::color::YELLOW_COLOR);
            view.addPlane(_tmp_plane, arvc::color::YELLOW_COLOR);
            view.addOrigin();
            view.show();
        }
        
        for (size_t i = 0; i < _tmp_plane_models.size()-1; i++)
        {

            float delta_d = abs(_tmp_plane_models[i].values[3] - _tmp_plane_models[i+1].values[3]);

            if (delta_d < this->width_threshold * 0.95){

                // JOIN MODELS
                pcl::PointIndicesPtr _new_inliers_1(new pcl::PointIndices);
                pcl::PointIndicesPtr _new_inliers_2(new pcl::PointIndices);

                _new_inliers_1 = this->get_inliers(_plane_models[i], 0.02, this->cloud_in_xyz);
                _new_inliers_2 = this->get_inliers(_plane_models[i+1], 0.02, this->cloud_in_xyz);

                _new_inliers_1->indices.insert(_new_inliers_1->indices.end(), _new_inliers_2->indices.begin(), _new_inliers_2->indices.end());

                tmp_indices = arvc::get_unique(_new_inliers_1->indices);
                tmp_coeffs = arvc::compute_planar_ransac_direction(this->cloud_in_xyz, tmp_indices, true, 0.02, 1000, _direction);

                if (cons.enable_debug){                
                    double dist = pcl::pointToPlaneDistanceSigned(pcl::PointXYZ(0,0,0), tmp_coeffs->values[0], tmp_coeffs->values[1], tmp_coeffs->values[2], tmp_coeffs->values[3]);

                    cons.debug(" - Mergin Planes " + to_string(i) + "-" + to_string(i+1) + " | Delta_d: " + to_string(delta_d) + " | Distance to origin: " + to_string(dist));

                    cout << " - New Plane Coeffs: ";
                    arvc::print_vector(tmp_coeffs->values);
                    
                    arvc::viewer view("TMP VIEWER");
                    arvc::plane _tmp_plane;
                    _tmp_plane.setPlane(tmp_coeffs, tmp_indices, this->cloud_in_xyz);
                    view.addCloud(_tmp_plane.cloud, arvc::color::YELLOW_COLOR);
                    view.addPlane(_tmp_plane, arvc::color::YELLOW_COLOR);
                    view.addOrigin();
                    view.show();
                }

                _tmp_plane_models[i+1] = *tmp_coeffs;

            }
            else{
                _new_plane_models.push_back(_tmp_plane_models[i]);
            }
        }
        _new_plane_models.push_back(_tmp_plane_models[_tmp_plane_models.size()-1]);

        cons.debug(" - Final plane models: " + to_string(_new_plane_models.size()));
        return _new_plane_models;
    }


    void remove_direction_planes(){
        cout << RED << "Entra a remove direction planes" << RESET << endl;
        pcl::PointIndicesPtr cat_indices(new pcl::PointIndices);

        for (arvc::plane plane : this->direction_planes){
            cat_indices->indices.insert(cat_indices->indices.end(), plane.inliers->indices.begin(), plane.inliers->indices.end());
        }
        
        arvc::remove_indices_from_cloud(this->cloud_in_xyz, cat_indices);
    }


    pcl::PointIndicesPtr get_inliers(const pcl::ModelCoefficients& _coeffs, const double& _threshold, const PointCloud::Ptr& _cloud){
    

        pcl::SampleConsensusModelPlane<PointT> sac_plane(_cloud);
        pcl::PointIndicesPtr _inliers(new pcl::PointIndices);

        Eigen::Vector4f _model(_coeffs.values[0], _coeffs.values[1], _coeffs.values[2], _coeffs.values[3]);

        sac_plane.selectWithinDistance(_model, _threshold, _inliers->indices);

        cons.info("Extracted Inliers: " + to_string(_inliers->indices.size()));

        return _inliers;
    }


    void get_intersection_indices(){

        pcl::PointIndicesPtr tmp_indices(new pcl::PointIndices);
        pcl::PointIndicesPtr cat_indices(new pcl::PointIndices);
        PointCloud::Ptr tmp_cloud(new PointCloud);
        pcl::IndicesPtr _indices(new pcl::Indices);

        cout << "\nGetting the intersection between " << this->plane_models.size() << " plane models..." << endl;
        
        for (size_t i = 0; i < this->plane_models.size(); i++){
            
            *tmp_indices = *this->get_inliers(this->plane_models[i], 0.02, this->cloud_in_xyz);
            *_indices = tmp_indices->indices;
            tmp_cloud = arvc::extract_indices(this->cloud_in_xyz, _indices);
            
            // arvc::viewer view("TMP VIEWER 2");
            // view.addCloud(tmp_cloud, arvc::color::WHITE_COLOR);
            // view.show();
            
            cat_indices->indices.insert(cat_indices->indices.end(), tmp_indices->indices.begin(), tmp_indices->indices.end());
        }

        this->intersection_indices->indices = arvc::get_duplicates(cat_indices->indices);

        cout << "Initial indices size: " << this->cloud_in_xyz->points.size() << endl;
        cout << "Intersection indices: " << this->intersection_indices->indices.size() << endl;
    }


    void get_intersection_cloud(){
        pcl::IndicesPtr indices(new pcl::Indices);
        *indices = this->intersection_indices->indices;
        this->cloud_intersection_xyz = arvc::extract_indices(this->cloud_in_xyz, indices);
    }


    void remove_intersection_indices(){
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(this->cloud_in_xyz);
        extract.setIndices(this->intersection_indices);
        extract.setNegative(true);
        // extract.filter(this->remain_indices->indices);
        extract.filter(*this->cloud_in_xyz);
    }

    /**
     * @brief Englobes functions to detect the initial plane and setup search directions
    */
    void initial_detection(){
        this->get_close_points();
        this->detect_initial_plane();
        this->compute_second_direction();
        this->compute_third_direction();
        this->get_search_directions();
    }

    
    void complete_detection(){
        
    // FILTER THE CLOUD SO RANSAC EXTRACTS PLANES FASTER AND BETTER
        // this->cloud_search_xyz = arvc::random_sample(this->cloud_in_xyz, 0.05);
        // this->cloud_search_xyz = arvc::uniform_sample(this->cloud_in_xyz, 0.05);
        this->cloud_search_xyz = arvc::voxel_filter(this->cloud_in_xyz, 0.02);

    // DETECT PRESENT PLANES IN EACH DIRECTION
        this->get_all_direction_planes();
        this->get_intersection_indices();
        this->get_intersection_cloud();
        this->remove_intersection_indices();
        // this->remove_plane_intersection_indices();
    }


/*     void grow_rectangle(const arvc::plane& _plane, const int& _num_samples){

        pcl::PointIndicesPtr rand_idx(new pcl::PointIndices);

        int max_value = _plane.inliers->indices.size();

        for (size_t i = 0; i < _num_samples; i++)
        {
            int random = rand() % max_value;
            rand_idx->indices.push_back(random);
        }

        pcl::PointXYZ seed = _plane.cloud->points[rand_idx->indices[0]];

        do
        {
            pcl::PointXYZ next_point;
            next_point.x = seed.x + 1;

            pcl::getPointsInBox(seed,);

        } while ();
    } */


/*   void remove_high_curvature(){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (this->cloud_in_xyz);
    ne.setSearchMethod (this->tree);
    ne.setRadiusSearch (0.03);
    ne.compute (*cloud_normals);

    // Remove points of cloud_in_xyz from the curvature value
    pcl::PointIndicesPtr new_idx (new pcl::PointIndices);
    pcl::Indices mis_indices;

    pcl::PassThrough<pcl::Normal> pass;
    
    pass.setInputCloud (cloud_normals);
    pass.setFilterFieldName ("curvature");
    pass.setFilterLimits (0.0, 0.8);
    // pass.setNegative (true);
    pass.filter(mis_indices);

    // Remove points of cloud_in_xyz from the curvature value
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (this->cloud_in_xyz);
    extract.setIndices (new_idx);
    // extract.setNegative (true);
    extract.filter (*this->cloud_in_xyz);
} */



    private:
        pcl::PointIndicesPtr remain_indices;
        pcl::PointIndicesPtr intersection_indices;
        vector<pcl::ModelCoefficients> plane_models;
        vector<pcl::PointIndices> plane_inliers;
        pcl::search::KdTree<PointT>::Ptr tree;
    

    // PUBLIC ATTRIBUTES
    public:
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_xyz;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_search_xyz;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_intersection_xyz;

        Eigen::Affine3f RobotBaseToSensorTF;
        float range_dist;

        arvc::plane initial_plane;
        bool initial_plane_found;

        vector<arvc::plane> detected_planes;
        vector<arvc::plane> direction_planes;

        Eigen::Vector3f first_direction;
        Eigen::Vector3f second_direction;
        Eigen::Vector3f third_direction;


        Eigen::Vector3f normal;
        stringstream ss;
        arvc::axes3d search_directions;

        int counter = 0;
        float length_threshold;
        float width_threshold;
        float min_length;
        float min_width;
        int min_plane_inliers;

};


int main(int argc, char const *argv[])
{
    // hide all mesages from pcl
    cons.enable = true;
    cons.enable_debug = true;
    cons.enable_warning = true;
    cons.enable_error = true;
    cons.enable_info = false;
    

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    std::cout << setprecision(3) << std::fixed;
    const double RANGE_DIST = 1;

    truss_plane_detector td(RANGE_DIST);

    td.length_threshold = 1.0;
    td.width_threshold = 0.25;
    td.min_length = 0.05;
    td.min_width = 0.05;
    td.min_plane_inliers = 200;

    td.RobotBaseToSensorTF.translation() = Eigen::Vector3f(0.1, 0.0, 0.3);
    td.RobotBaseToSensorTF.linear() = Eigen::Quaternionf::Identity().toRotationMatrix();

    // COMMENT OPTION DEPENDING ON THE COMPUTER YOU ARE USING
    td.cloud_in_xyz = arvc::readCloud("../examples/example_cloud.pcd");

    // FILTERS TO APPLY TO THE INPUT CLOUD
    td.cloud_in_xyz = arvc::remove_origin_points(td.cloud_in_xyz);

    // APPLY TRANSFORMATION TO THE ROBOT BASE
    pcl::transformPointCloud(*td.cloud_in_xyz, *td.cloud_in_xyz, td.RobotBaseToSensorTF);



    // DETECT INITIAL PLANE
    td.initial_detection();


    // DETECT PLANES IN THE REMAINING CLOUD IN THE THREE DIRECTIONS
    td.complete_detection();
    
    
    // VISUALIZE RESULTS
    arvc::viewer view("VIEWER");

    // for (arvc::plane plane : td.detected_planes)
    //     view.addCloud(plane.cloud, plane.color);
    
    view.addCloud(td.cloud_intersection_xyz, arvc::color(255,0,0));

    view.addOrigin();
    view.show();


    
    return 0;
}



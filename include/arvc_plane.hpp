#pragma once
#include <iostream>
#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

class plane
{
public:

    plane(){
    this->coeffs.reset(new pcl::ModelCoefficients);
    this->inliers.reset(new pcl::PointIndices);
    this->cloud.reset(new PointCloud);
    this->original_cloud.reset(new PointCloud);

    this->coeffs->values = {0,0,0,0};
    this->inliers->indices = {0};
    this->normal = Eigen::Vector3f(0,0,0);
    this->polygon = vector<Eigen::Vector3f>(5);

    this->projected_cloud.reset(new PointCloud);

    this->length = 0;
    this->width = 0;
    };

/*       plane(pcl::ModelCoefficientsPtr _coeffs, pcl::PointIndicesPtr _indices)
    {
    this->coeffs.reset(new pcl::ModelCoefficients);
    this->inliers.reset(new pcl::PointIndices);

    *this->coeffs = *_coeffs;
    *this->inliers = *_indices;
    
    cout << "PLANE OBJ INLIERS SIZE: " << this->inliers->indices.size() << endl;
    cout << "PLANE OBJ COEFFS: " << *this->coeffs << endl;
    cout << "-----------------------------" << endl;
    }; */

    ~plane(){
    this->coeffs->values = {0,0,0,0};
    this->inliers->indices = {0};
    };

    friend std::ostream& operator<<(std::ostream& os, const arvc::plane& p)
    {
    os << "Plane parameters: [ " << p.coeffs->values[0] << ", " << p.coeffs->values[1] << ", " << p.coeffs->values[2] << ", " << p.coeffs->values[3] << " ]" << endl;
    os << "Plane inliers: " << p.inliers->indices.size() << endl;

    return os;
    }

    void setPlane(pcl::ModelCoefficientsPtr _coeffs, pcl::PointIndicesPtr _indices, PointCloud::Ptr _cloud_in){
    *this->coeffs = *_coeffs;
    *this->inliers = *_indices;
    *this->original_cloud = *_cloud_in;
    this->getNormal();
    this->getCloud();
    this->getEigenVectors();
    this->getEigenValues();
    this->getCentroid();
    this->getTransform();  
    this->getPolygon();
    this->color.random();
    };

    PointCloud::Ptr getCloud(){
    pcl::ExtractIndices<PointT> extract;
    
    extract.setInputCloud(this->original_cloud);
    extract.setIndices(this->inliers);
    extract.setNegative(false);
    extract.filter(*this->cloud);

    return this->cloud;
    };
    
    Eigen::Vector3f getNormal(){
    this->normal = Eigen::Vector3f(this->coeffs->values[0], this->coeffs->values[1], this->coeffs->values[2]);
    return normal;
    }

    void getEigenVectors(){
    this->eigenvectors = arvc::compute_eigenvectors3D(this->cloud, false);
    }

    void getEigenValues(){
    this->eigenvalues = arvc::compute_eigenvalues3D(this->cloud);
    }

    void projectOnPlane(){

    pcl::ProjectInliers<PointT> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(this->cloud);
    proj.setModelCoefficients(this->coeffs);
    proj.filter(*this->projected_cloud);
    }

    void getCentroid(){
    pcl::compute3DCentroid(*this->cloud, this->centroid);
    }


    void getPolygon(){
    PointCloud::Ptr relative_cloud(new PointCloud);

    this->projectOnPlane();
    pcl::transformPointCloud(*this->projected_cloud, *relative_cloud, this->tf.inverse());

    PointT max_point;
    PointT min_point;

    pcl::getMinMax3D(*relative_cloud, min_point, max_point);

    this->polygon[0] = Eigen::Vector3f(min_point.x, min_point.y, 0.0);
    this->polygon[1] = Eigen::Vector3f(min_point.x, max_point.y, 0.0);
    this->polygon[2] = Eigen::Vector3f(max_point.x, max_point.y, 0.0);
    this->polygon[3] = Eigen::Vector3f(max_point.x, min_point.y, 0.0);
    this->polygon[4] = this->polygon[0];

    this->length = abs(this->polygon[1].x() - this->polygon[2].x()); 
    this->width = abs(this->polygon[0].y() - this->polygon[1].y());

    for (int i = 0; i < this->polygon.size(); i++)
        this->polygon[i] = this->tf * this->polygon[i];
    }


    void getTransform(){
        this->tf.translation() = this->centroid.head<3>();
        this->tf.linear() = this->eigenvectors.getRotationMatrix();
    }

    pcl::ModelCoefficientsPtr coeffs;
    pcl::PointIndicesPtr inliers;
    
    PointCloud::Ptr cloud;
    PointCloud::Ptr original_cloud;
    Eigen::Vector3f normal;
    Eigen::Vector4f centroid;
    Eigen::Affine3f tf;
    vector<pcl::PointIndices> clusters;

    arvc::axes3d eigenvectors;
    Eigen::Vector3f eigenvalues;
    
    vector<Eigen::Vector3f> polygon;
    float length;
    float width;
    
    arvc::color color;

    private:
        PointCloud::Ptr projected_cloud;
    

};

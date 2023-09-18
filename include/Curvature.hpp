// C++

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/eigen.h>
#include <pcl/common/centroid.h>
#include <Eigen/Dense>

// DEFINES 
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"  
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"

// TYPE DEFINITIONS
// typedef pcl::PointXYZ PointT;
// typedef pcl::PointCloud<PointT> PointCloud;

// NAMESPACES

namespace fjs{

  template<typename PointT>
  class Curvature{
    private:
      Eigen::Vector4d centroid_;
      typename pcl::PointCloud<PointT>::Ptr cloud_;
      pcl::PointIndices::Ptr indices_;
      Eigen::Matrix3d covariance_matrix_;
      Eigen::Vector3f eigen_values_;
      Eigen::Matrix3f eigen_vectors_;
      float curvature_;
      int knn_;
      float radius_;

    public:
      Curvature();
      ~Curvature();

      void setCentroid(pcl::PointXYZ centroid) 
      { 
        this->centroid_[0] = centroid.x;
        this->centroid_[1] = centroid.y;
        this->centroid_[2] = centroid.z;
        this->centroid_[3] = 1.0;
      
      }
      void setInputCloud(typename pcl::PointCloud<PointT>::Ptr cloud) { this->cloud_ = cloud; }
      void setIndices(pcl::PointIndices::Ptr indices) { this->indices_ = indices; }
      pcl::PointIndices::Ptr getIndices() { return this->indices_; }

      void set_knn(int knn);
      void set_radius(float radius);

      void chek_input()
      {
        if (this->cloud_ == nullptr)
        {
          std::cout << RED << "Input cloud is not set!" << RESET << std::endl;
          return;
        }
      }

      void computeCovarianceMatrix()
      {
        if (this->indices_ == nullptr)
          pcl::computeCovarianceMatrixNormalized(*this->cloud_, this->centroid_, this->covariance_matrix_);
        else
          pcl::computeCovarianceMatrixNormalized(*this->cloud_, *this->indices_, this->centroid_, this->covariance_matrix_);
      }

      void computePCA()
      {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(this->covariance_matrix_, Eigen::ComputeEigenvectors);
        this->eigen_values_ = eigen_solver.eigenvalues();
        this->eigen_vectors_ = eigen_solver.eigenvectors(); //.cast<float>();
      }

      float computeCurvature()
      {
        this->computeCovarianceMatrix();
        this->computePCA();
        this->curvature_ = this->eigen_values_[0] / (this->eigen_values_[0] + this->eigen_values_[1] + this->eigen_values_[2]);
        return this->curvature_;
      }
  };
}





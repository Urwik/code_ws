// cpp
#include <iostream>

// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_representation.h>

  // PCL FILTERS
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>


pcl::PointCloud<pcl::PointXYZL>::Ptr parseIntensityToLabel(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in)
{
  pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZL>);
  cloud_out->points.resize(cloud_in->points.size());

  for (size_t i = 0; i < cloud_in->size(); i++)
  {
    cloud_out->points[i].x = cloud_in->points[i].x;
    cloud_out->points[i].y = cloud_in->points[i].y;
    cloud_out->points[i].z = cloud_in->points[i].z;
    cloud_out->points[i].label = (uint32_t) cloud_in->points[i].intensity;
  }

  return cloud_out;  
}


pcl::ModelCoefficients::Ptr applyRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const bool optimizeCoefs,
                                        float distThreshold = 0.03, int maxIterations = 1000)
{
  pcl::SACSegmentation<pcl::PointXYZ> ransac;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices inliers;

  ransac.setInputCloud(cloud);
  ransac.setOptimizeCoefficients(optimizeCoefs);
  ransac.setModelType(pcl::SACMODEL_PLANE);
  ransac.setMethodType(pcl::SAC_RANSAC);
  ransac.setMaxIterations(maxIterations);
  ransac.setDistanceThreshold(distThreshold);
  ransac.segment(inliers, *coefficients);

  return coefficients;
}


Eigen::Matrix<float,1,6> computeBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  // Compute principal directions
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid(*cloud, pcaCentroid);
  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();

  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
  /// the signs are different and the box doesn't get correctly oriented in some cases.
  eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  
                                                                                  

  // Transform the original cloud to the origin where the principal components correspond to the axes.
  Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
  projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
  projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cloud, *cloudPointsProjected, projectionTransform);
  // Get the minimum and maximum points of the transformed cloud.
  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
  const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

  // Final transform
  const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
  const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

}


int main(int argc, char **argv)
{

  
  return 0;
}




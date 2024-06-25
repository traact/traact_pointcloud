/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef traact_pointcloud_SRC_TRAACT_COMPONENT_CUDACOMPONENTFUNCTIONS_CUH_
#define traact_pointcloud_SRC_TRAACT_COMPONENT_CUDACOMPONENTFUNCTIONS_CUH_

#include <opencv2/cudaimgproc.hpp>
#include <cuda.h>
#include <open3d/geometry/PointCloud.h>
#include <traact/vision_datatypes.h>
#include <Eigen/Core>
namespace traact::vision {

void createPointCloud(const cv::cuda::GpuMat &depth,
                      const cv::cuda::GpuMat &xy_table,
                      float depth_scale,
                      cv::cuda::GpuMat &point_cloud,
                      cudaStream_t stream);

void colorPointCloud(const cv::cuda::GpuMat &point_cloud, const cv::cuda::GpuMat &color_image, const vision::CameraCalibration& calibration, const Eigen::Affine3f& color_to_depth, cv::cuda::GpuMat &color_points, cudaStream_t stream );

} // traact

#endif //traact_pointcloud_SRC_TRAACT_COMPONENT_CUDACOMPONENTFUNCTIONS_CUH_

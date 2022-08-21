/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "CudaComponentFunctions.cuh"
#include <opencv2/core/cuda/common.hpp>

namespace traact::vision {

__global__ void createPointCloud(float depth_scale,
                                 cv::cuda::PtrStepSz<uint16_t> depth,
                                 cv::cuda::PtrStepSz<cv::Vec2f> xy_table,
                                 cv::cuda::PtrStepSz<cv::Vec4f> point_cloud) {

    auto x = blockDim.x * blockIdx.x + threadIdx.x;
    auto y = blockDim.y * blockIdx.y + threadIdx.y;

    if (x >= depth.cols || y >= depth.rows) {
        return;
    }

    float z = depth(y, x) / depth_scale;

    auto &point = point_cloud(y, x);
    auto &xy_factor = xy_table(y, x);

    point.val[0] = xy_factor.val[0] * z;
    point.val[1] = xy_factor.val[1] * z;
    point.val[2] = z;
    point.val[3] = 1.0f;

}

__global__ void colorPointCloud(const cv::cuda::PtrStepSz<cv::Vec4f> point_cloud,
                                const cv::cuda::PtrStepSz<cv::Vec4b> color_image,
                                const Eigen::Affine3f color_to_depth,
                                cv::cuda::PtrStepSz<cv::Vec4b> color_points,
                                const int width, const int height,
                                const float fx, const float fy,
                                const float cx, const float cy,
                                const float k1,const float k2, const float k3,const float k4, const float k5, const float k6,
                                const float p1,const float p2
                                ) {

    auto x = blockDim.x * blockIdx.x + threadIdx.x;
    auto y = blockDim.y * blockIdx.y + threadIdx.y;

    if (x >= point_cloud.cols || y >= point_cloud.rows) {
        return;
    }

    auto& point_depth = point_cloud(y, x);
    auto &point_color = color_points(y,x);

    if(point_depth.val[2] < 0.01){
        point_color.val[0] = 255;
        point_color.val[1] = 0;
        point_color.val[2] = 0;
        return;
    }

    Eigen::Vector3f point =  color_to_depth * Eigen::Vector3f(point_depth.val[0], point_depth.val[1], point_depth.val[2]);

    // from k4a sdk transformation_project_internal
    float codx = 0.; // center of distortion is set to 0 for Brown Conrady model
    float cody = 0.;

    float xp = point.x() / point.z() - codx;
    float yp = point.y() / point.z() - cody; // flip on y-axis due to coordinate change image vs. opengl

    float xp2 = xp * xp;
    float yp2 = yp * yp;
    float xyp = xp * yp;
    float rs = xp2 + yp2;

    float rss = rs * rs;
    float rsc = rss * rs;
    float a = 1.f + k1 * rs + k2 * rss + k3 * rsc;
    float b = 1.f + k4 * rs + k5 * rss + k6 * rsc;
    float bi;
    if (b != 0.f)
    {
        bi = 1.f / b;
    }
    else
    {
        bi = 1.f;
    }
    float d = a * bi;

    float xp_d = xp * d;
    float yp_d = yp * d;

    float rs_2xp2 = rs + 2.f * xp2;
    float rs_2yp2 = rs + 2.f * yp2;

    xp_d += rs_2xp2 * p2 + 2.f * xyp * p1;
    yp_d += rs_2yp2 * p1 + 2.f * xyp * p2;

    float xp_d_cx = xp_d + codx;
    float yp_d_cy = yp_d + cody;

    float u = xp_d_cx * fx + cx;
    float v = yp_d_cy * fy + cy;

    int image_x = u;
    int image_y = v;
    if(image_x < 0){
        image_x = 0;
    }
    if(image_x >= color_image.cols){
        image_x = color_image.cols-1;
    }
    if(image_y < 0){
        image_y = 0;
    }
    if(image_y >= color_image.rows){
        image_y = color_image.rows-1;
    }



    const auto &image_color = color_image(image_y, image_x);
    point_color.val[0] = image_color.val[2];
    point_color.val[1] = image_color.val[1];
    point_color.val[2] = image_color.val[0];

}

void createPointCloud(const cv::cuda::GpuMat &depth,
                      const cv::cuda::GpuMat &xy_table,
                      float depth_scale,
                      cv::cuda::GpuMat &point_cloud,
                      cudaStream_t stream) {

    auto width = depth.cols;
    auto height = depth.rows;

    const dim3 block(16, 16, 1);
    const dim3 grid(cv::cuda::device::divUp(width, block.x),
                    cv::cuda::device::divUp(height, block.y), 1);

    createPointCloud<<<grid, block, 0, stream>>>(depth_scale,
                                                 depth,
                                                 xy_table,
                                                 point_cloud);

}
void colorPointCloud(const cv::cuda::GpuMat &point_cloud,
                     const cv::cuda::GpuMat &color_image,
                     const CameraCalibration &calibration,
                     const Eigen::Affine3f &color_to_depth,
                     cv::cuda::GpuMat &color_points,
                     cudaStream_t stream) {
    auto width = point_cloud.cols;
    auto height = point_cloud.rows;

    const dim3 block(16, 16, 1);
    const dim3 grid(cv::cuda::device::divUp(width, block.x),
                    cv::cuda::device::divUp(height, block.y), 1);


//    Eigen::Vector3f euler_angles = color_to_depth.rotation().eulerAngles(0,1,2);
//
//    Eigen::Matrix3f rotation = (Eigen::AngleAxisf( euler_angles[0], Eigen::Vector3f::UnitX())
//        * Eigen::AngleAxisf(-euler_angles[1], Eigen::Vector3f::UnitY()) // invert rotation direction around y-axis
//        * Eigen::AngleAxisf(-euler_angles[2], Eigen::Vector3f::UnitZ())).matrix(); // invert rotation direction around y-axis
//    auto out = Eigen::Quaternionf(rotation);
//
//    out = { params.translation[0] / units_per_meter,
//            -params.translation[1] / units_per_meter,
//            -params.translation[2] / units_per_meter
//    };

    colorPointCloud<<<grid, block, 0, stream>>>(point_cloud, color_image, color_to_depth, color_points,
                                                calibration.width, calibration.height,
                                                calibration.fx, calibration.fy,
                                                calibration.cx, calibration.cy,
                                                calibration.radial_distortion[0], calibration.radial_distortion[1],
                                                calibration.radial_distortion[2], calibration.radial_distortion[3],
                                                calibration.radial_distortion[4], calibration.radial_distortion[5],
                                                calibration.tangential_distortion[0], calibration.tangential_distortion[1]);

}
} // traact
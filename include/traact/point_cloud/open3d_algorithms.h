/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACT_COMPONENT_POINTCLOUD_SRC_TRAACT_POINT_CLOUD_OPEN3D_ALGORITHMS_H_
#define TRAACT_COMPONENT_POINTCLOUD_SRC_TRAACT_POINT_CLOUD_OPEN3D_ALGORITHMS_H_

/**
 * algorithms from the open3d examples and tutorials
 */

//#include <open3d/Open3D.h>
#include <spdlog/spdlog.h>
#include "point_cloud_datatypes.h"
#include <open3d/pipelines/registration/PoseGraph.h>

namespace traact::point_cloud {

bool downSample(const PointCloud &cloud, PointCloud &output, const double voxel_size);

void estimateNormals(PointCloud &cloud, double voxel_size, const Eigen::Vector3d &camera_pos);

struct MultiStageColorIcpConfig {
    PointCloud main_cloud{nullptr};
    PointCloud sub_cloud{nullptr};
    Eigen::Matrix4d sub_pose{Eigen::Matrix4d::Identity()};
    bool estimate_main_normals{false};
    bool estimate_sub_normals{false};
    Eigen::Vector3d main_pos{Eigen::Vector3d::Identity()};
    bool validate();
    bool validate(const PointCloud& cloud, const std::string& name, bool& estimate_normals);
};

Eigen::Matrix4d multiStageColorIcp(MultiStageColorIcpConfig &config);

struct PoseGraphEdge {
    PoseGraphEdge(size_t source_id, size_t target_id, bool uncertain);
    size_t source_id;
    size_t target_id;
    bool uncertain;
};

struct MultiwayRegistrationConfig {
    MultiwayRegistrationConfig(size_t count_cameras, double max_coarse, double max_fine);
    double max_correspondence_distance_coarse;
    double max_correspondence_distance_fine;
    std::vector<PointCloud> point_clouds;
    std::vector<Eigen::Matrix4d> origin_to_camera;
    std::vector<PoseGraphEdge> edges;

};

open3d::pipelines::registration::PoseGraph multiwayRegistration(MultiwayRegistrationConfig &config);

std::pair<Eigen::Matrix4d, Eigen::Matrix6d> pairwiseRegistration(const PointCloud &source,
                                                                 const PointCloud &target,
                                                                 double max_correspondence_distance_coarse,
                                                                 double max_correspondence_distance_fine,
                                                                 const Eigen::Matrix4d &target_to_source_estimate);

} // traact

#endif //TRAACT_COMPONENT_POINTCLOUD_SRC_TRAACT_POINT_CLOUD_OPEN3D_ALGORITHMS_H_

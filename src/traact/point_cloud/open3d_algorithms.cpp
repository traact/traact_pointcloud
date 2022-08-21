/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "open3d_algorithms.h"

namespace traact::point_cloud {

bool downSample(const PointCloud &cloud, PointCloud &output, const double voxel_size) {
    output = cloud->VoxelDownSample(voxel_size);
    if (!output) {
        SPDLOG_ERROR("VoxelDownSample failed");
        return false;
    }
    return true;
}

void estimateNormals(PointCloud &cloud, double voxel_size, const Eigen::Vector3d &camera_pos) {
    open3d::geometry::KDTreeSearchParamHybrid normals_params(voxel_size*2.0, 30);
    const bool fast_normal_computation = true;
    cloud->EstimateNormals(normals_params, fast_normal_computation);
    cloud->OrientNormalsTowardsCameraLocation(camera_pos);
}

Eigen::Matrix4d multiStageColorIcp(MultiStageColorIcpConfig &config) {

    if(!config.validate()) {
        return Eigen::Matrix4d::Identity();
    }
    Eigen::Matrix4d new_origin_to_sub = config.sub_pose;

    std::vector<double> voxel_size_stage{0.05, 0.05 / 2, 0.05 / 4};
    std::vector<int> max_iter{50, 30, 14};
    for (int stage = 0; stage < 3; ++stage) {
        const double voxel_size = voxel_size_stage[stage];
        const int iter = max_iter[stage];
        SPDLOG_INFO("voxel size: {0}  Max iterations: {1} ", voxel_size_stage[stage], max_iter[stage]);

        point_cloud::PointCloud main_cloud_down;
        if (!downSample(config.main_cloud, main_cloud_down, voxel_size)) {
            return Eigen::Matrix4d::Identity();
        }

        if(config.estimate_main_normals){
            estimateNormals(main_cloud_down, voxel_size, config.main_pos);
        }


        point_cloud::PointCloud sub_cloud_down;
        Eigen::Affine3d new_pose(new_origin_to_sub);
        if (!downSample(config.sub_cloud, sub_cloud_down, voxel_size)) {
            return Eigen::Matrix4d::Identity();
        }
        if(config.estimate_sub_normals){
            estimateNormals(sub_cloud_down, voxel_size, Eigen::Vector3d());
        }

        open3d::pipelines::registration::ICPConvergenceCriteria criteria(1e-6, 1e-6, iter);
        // How much it tends towards using the geometry instead of the color
        const double lambda_geometric = 0.968;
        open3d::pipelines::registration::TransformationEstimationForColoredICP transform_estimate(lambda_geometric);

        auto result = open3d::pipelines::registration::RegistrationColoredICP(
            *sub_cloud_down,
            *main_cloud_down,
            voxel_size_stage[stage],
            new_origin_to_sub,
            transform_estimate,
            criteria);

        new_origin_to_sub = result.transformation_;

        SPDLOG_INFO("color icp result: fitness {0}, inlier rmse {1}", result.fitness_, result.inlier_rmse_);

    }

    return new_origin_to_sub;
}

open3d::pipelines::registration::PoseGraph multiwayRegistration(MultiwayRegistrationConfig &config) {

    SPDLOG_INFO("start multiway registration");
    open3d::pipelines::registration::PoseGraph pose_graph;

    // origin node
    pose_graph.nodes_.emplace_back(Eigen::Matrix4d::Identity());
    for (auto i = 0; i < config.point_clouds.size(); ++i) {
        pose_graph.nodes_.emplace_back(config.origin_to_camera[i]);
        pose_graph.edges_.emplace_back(0, i+1,config.origin_to_camera[i], Eigen::Matrix6d::Identity(), true);
    }

    for(const auto&[source_id, target_id, uncertain] : config.edges){
        SPDLOG_INFO("apply point-to-plain ICP {0} : {1}", source_id+1, target_id+1);
        Eigen::Matrix4d target_to_source = config.origin_to_camera[target_id].inverse() * config.origin_to_camera[source_id] ;
        auto [transformation_icp, information_icp] = pairwiseRegistration(config.point_clouds[source_id],
                                                                          config.point_clouds[target_id],
                                                                          config.max_correspondence_distance_coarse,
                                                                          config.max_correspondence_distance_fine,
                                                                          target_to_source);
        pose_graph.edges_.emplace_back(source_id+1, target_id+1, transformation_icp, information_icp, uncertain);
    }

    SPDLOG_INFO("end multiway registration");

    return pose_graph;
}
std::pair<Eigen::Matrix4d, Eigen::Matrix6d> pairwiseRegistration(const PointCloud &source,
                                                                 const PointCloud &target,
                                                                 double max_correspondence_distance_coarse,
                                                                 double max_correspondence_distance_fine,
                                                                 const Eigen::Matrix4d &target_to_source_estimate) {

    MultiStageColorIcpConfig config;
    config.main_cloud = target;
    config.sub_cloud = source;
    config.sub_pose = target_to_source_estimate;

    auto icp_target_to_source = multiStageColorIcp(config);

    //auto icp_coarse = open3d::pipelines::registration::RegistrationICP(*source, *target, max_correspondence_distance_coarse, target_to_source_estimate, open3d::pipelines::registration::TransformationEstimationPointToPlane());
    //auto icp_fine = open3d::pipelines::registration::RegistrationICP(*source, *target, max_correspondence_distance_fine, icp_coarse.transformation_, open3d::pipelines::registration::TransformationEstimationPointToPlane());
    auto icp_information = open3d::pipelines::registration::GetInformationMatrixFromPointClouds(*source, *target, max_correspondence_distance_fine, icp_target_to_source);

    return {icp_target_to_source, icp_information};
}

MultiwayRegistrationConfig::MultiwayRegistrationConfig(size_t count_cameras, double max_coarse, double max_fine) : max_correspondence_distance_coarse(max_coarse), max_correspondence_distance_fine(max_fine){
    point_clouds.resize(count_cameras);
    origin_to_camera.resize(count_cameras);

}
bool MultiStageColorIcpConfig::validate(){

    if(!validate(main_cloud,"main_cloud", estimate_main_normals)){
        return false;
    };
    if(!validate(main_cloud,"sub_cloud", estimate_sub_normals)){
        return false;
    };

    return true;
}
bool MultiStageColorIcpConfig::validate(const PointCloud& cloud, const std::string& name, bool& estimate_normals) {
    if(!cloud){
        SPDLOG_ERROR("{0} must not be null", name);
        return false;
    }
    if(cloud->points_.empty()){
        SPDLOG_ERROR("{0} points must not be empty", name);
        return false;
    }
    if(cloud->colors_.empty()){
        SPDLOG_ERROR("{0} colors must not be empty", name);
        return false;
    }
    if(cloud->points_.size() != cloud->colors_.size()){
        SPDLOG_ERROR("{0} points and colors must have same size", name);
        return false;
    }
    if(cloud->normals_.empty() && !estimate_normals){
        SPDLOG_WARN("main_cloud, color icp needs normals, but estimation was disabled and cloud has none, enabling estimation");
        estimate_normals = true;
    }
    return true;
}
PoseGraphEdge::PoseGraphEdge(size_t source_id, size_t target_id, bool uncertain)
    : source_id(source_id), target_id(target_id), uncertain(uncertain) {}
} // traact
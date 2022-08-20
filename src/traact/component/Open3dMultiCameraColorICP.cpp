/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/opencv/OpenCVUtils.h>
#include <traact/vision.h>
#include <traact/pointCloud.h>

#include <traact/math/perspective.h>
#include <open3d/Open3D.h>

namespace traact::component {

class Open3dMultiCameraColorICP : public Component {
 public:
    using InPortMainCameraPose = buffer::PortConfig<spatial::Pose6DHeader, 0>;
    using InPortMainCameraCloud = buffer::PortConfig<pointCloud::PointCloudHeader, 1>;

    using InPortGroupSubCameraPose = buffer::PortConfig<traact::spatial::Pose6DHeader, 0>;
    using InPortGroupSubCameraCloud = buffer::PortConfig<pointCloud::PointCloudHeader, 1>;
    using OutPortGroupSubCameraPose = buffer::PortConfig<traact::spatial::Pose6DHeader, 0>;

    explicit Open3dMultiCameraColorICP(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("Open3dMultiCameraColorICP",
                                                       Concurrency::SERIAL,
                                                       ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPortMainCameraPose>("input_pose")
            .addConsumerPort<InPortMainCameraCloud>("input_cloud")
            .beginPortGroup("sub_camera", 1)
            .addConsumerPort<InPortGroupSubCameraPose>("input_pose")
            .addConsumerPort<InPortGroupSubCameraCloud>("input_cloud")
            .addProducerPort<OutPortGroupSubCameraPose>("output")
            .endPortGroup()
            .addParameter("max_distance", 0.02, 0.0, 1.0);
        return pattern;
    }

    void configureInstance(const pattern::instance::PatternInstance &pattern_instance) override {
        cameras_info_ = pattern_instance.getPortGroupInfo("sub_camera");
    }

    virtual bool configure(const pattern::instance::PatternInstance &pattern_instance,
                           buffer::ComponentBufferConfig *data) override {
        pattern_instance.setValueFromParameter("max_distance", max_distance_);

        return true;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {

        if (cameras_info_.size < 1) {
            SPDLOG_WARN("need at least 1 sub camera, error in dataflow setup");
            return true;
        }

        std::vector<bool> sub_processed(cameras_info_.size, false);

        const auto &main_pose = data.getInput<InPortMainCameraPose>();

        // copy point clouds to apply init transform
        // integrate all other sub clouds into it
        open3d::geometry::PointCloud main_cloud = *data.getInput<InPortMainCameraCloud>();

        main_cloud.Transform(main_pose.matrix().cast<double>());
        auto estimated_size = main_cloud.points_.size() * (cameras_info_.size + 1);
        main_cloud.points_.reserve(estimated_size);
        estimateNormals(main_cloud, 0.01, main_pose.cast<double>().translation());



        auto process_order = createProcessOrder(data);

        for (size_t order_index = 0; order_index < process_order.size(); ++order_index) {
            size_t sub_index = process_order.at(order_index);

            const auto
                &sub_pose_float = data.getInput<InPortGroupSubCameraPose>(cameras_info_.port_group_index, sub_index);
            open3d::geometry::PointCloud
                sub_cloud = *data.getInput<InPortGroupSubCameraCloud>(cameras_info_.port_group_index, sub_index);
            Eigen::Matrix4d sub_pose = sub_pose_float.matrix().cast<double>();

            estimateNormals(sub_cloud, 0.01, Eigen::Vector3d());
            SPDLOG_INFO("color icp for sub camera index {0} ", sub_index);

            Eigen::Matrix4d sub_pose_new = colorIcp(main_cloud, sub_cloud, sub_pose, Eigen::Vector3d());

            sub_cloud.Transform(sub_pose_new);

            for (auto i = 0; i < sub_cloud.points_.size(); ++i) {
                main_cloud.points_.push_back(sub_cloud.points_[i]);
                main_cloud.colors_.push_back(sub_cloud.colors_[i]);
            }
            data.getOutput<OutPortGroupSubCameraPose>(cameras_info_.port_group_index, sub_index) =
                spatial::Pose6D(sub_pose_new.cast<float>());

        }
        open3d::io::WritePointCloud("fused_cloud.ply", main_cloud);
        return true;
    }
    void estimateNormals(open3d::geometry::PointCloud &cloud, double radius, Eigen::Vector3d camera_pos) {
        const double normal_radius = radius * 2.0;
        open3d::geometry::KDTreeSearchParamHybrid normals_params(normal_radius, 30);
        const bool fast_normal_computation = true;
        cloud.EstimateNormals(normals_params, fast_normal_computation);
        // Incorporate the assumption that normals should be pointed towards the camera
        cloud.OrientNormalsTowardsCameraLocation(camera_pos);

    }
    std::vector<size_t> createProcessOrder(buffer::ComponentBuffer &data) {

        std::vector<std::pair<size_t, float>> sub_diff;
        sub_diff.reserve(cameras_info_.size);

        const auto &main_pose = data.getInput<InPortMainCameraPose>();
        Eigen::Vector3f main_pos = main_pose.translation();

        for (size_t sub_index = 0; sub_index < cameras_info_.size; ++sub_index) {

            const auto &sub_pose = data.getInput<InPortGroupSubCameraPose>(cameras_info_.port_group_index, sub_index);
            Eigen::Vector3f sub_pos = sub_pose.translation();

            auto diff = std::atan2(main_pos.cross(sub_pos).norm(), main_pos.dot(sub_pos));
            sub_diff.emplace_back(sub_index, diff);
        }
        std::vector<size_t> process_order;
        std::sort(sub_diff.begin(), sub_diff.end(), [](const auto &value_1, const auto &value_2) {
            return value_1.second < value_2.second;
        });
        for (const auto &index_diff : sub_diff) {
            process_order.emplace_back(index_diff.first);

        }

        return process_order;
    }
    Eigen::Matrix4d colorIcp(const open3d::geometry::PointCloud &main_cloud,
                             const open3d::geometry::PointCloud &sub_cloud,
                             const Eigen::Matrix4d &sub_pose,
                             const Eigen::Vector3d main_pos) {

        Eigen::Matrix4d new_origin_to_sub = sub_pose;


        // from open3d example GeneralizedICP.cpp
        std::vector<double> voxel_radius{0.04, 0.02, 0.01};
        std::vector<int> max_iter{50, 30, 14};
        for (int stage = 0; stage < 3; ++stage) {
            const double radius = voxel_radius[stage];
            const int iter = max_iter[stage];
            SPDLOG_INFO("voxel radius: {0}  Max iterations: {1} ", voxel_radius[stage], max_iter[stage]);

            pointCloud::PointCloud main_cloud_down;
            if (!downSample(main_cloud, main_cloud_down, radius, main_pos)) {
                return Eigen::Matrix4d::Identity();
            }


            pointCloud::PointCloud sub_cloud_down;
            Eigen::Affine3d new_pose(new_origin_to_sub);
            if (!downSample(sub_cloud, sub_cloud_down, radius, new_pose.translation())) {
                return Eigen::Matrix4d::Identity();
            }

            open3d::pipelines::registration::ICPConvergenceCriteria criteria(1e-6, 1e-6, iter);
            // How much it tends towards using the geometry instead of the color
            const double lambda_geometric = 0.968;
            open3d::pipelines::registration::TransformationEstimationForColoredICP transform_estimate(lambda_geometric);

            auto result = open3d::pipelines::registration::RegistrationColoredICP(
                *sub_cloud_down,
                *main_cloud_down,
                voxel_radius[stage],
                new_origin_to_sub,
                transform_estimate,
                criteria);

            new_origin_to_sub = result.transformation_;

            SPDLOG_INFO("color icp result: fitness {0}, inlier rmse {1}", result.fitness_, result.inlier_rmse_);

        }




        return new_origin_to_sub;
    }
    bool downSample(const open3d::geometry::PointCloud &cloud,
                    pointCloud::PointCloud& output,
                    const double radius,
                    const Eigen::Vector3d &camera_pos) {
        output = cloud.VoxelDownSample(radius);;
        if (!output) {
            SPDLOG_ERROR("VoxelDownSample failed");
            return false;
        }
        //estimateNormals(*output, radius, camera_pos);
        return true;
    }

 private:
    PortGroupInfo cameras_info_;
    double max_distance_;

};

CREATE_TRAACT_COMPONENT_FACTORY(Open3dMultiCameraColorICP)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::Open3dMultiCameraColorICP)
END_TRAACT_PLUGIN_REGISTRATION

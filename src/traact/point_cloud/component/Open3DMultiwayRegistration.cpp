/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/vision.h>
#include "traact/point_cloud.h"

#include <open3d/Open3D.h>
#include "traact/point_cloud/open3d_algorithms.h"
#include <open3d/pipelines/registration/GlobalOptimization.h>

namespace traact::component {

class Open3DMultiwayRegistration : public Component {
 public:
    using InPortGroupCameraPose = buffer::PortConfig<traact::spatial::Pose6DHeader, 0>;
    using InPortGroupCameraCloud = buffer::PortConfig<point_cloud::PointCloudHeader, 1>;
    using OutPortGroupCameraPose = buffer::PortConfig<traact::spatial::Pose6DHeader, 0>;

    explicit Open3DMultiwayRegistration(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("Open3DMultiwayRegistration",
                                                       Concurrency::SERIAL,
                                                       ComponentType::SYNC_FUNCTIONAL);

        pattern->beginPortGroup("camera", 2)
            .addConsumerPort<InPortGroupCameraPose>("input_pose")
            .addConsumerPort<InPortGroupCameraCloud>("input_cloud")
            .addProducerPort<OutPortGroupCameraPose>("output")
            .endPortGroup()
            .addParameter("reference_node", 0, 0, 10)
            .addParameter("edge_prune_threshold", 0.25, 0.0, 1.0)
            .addParameter("preference_loop_closure", 1.0, 0.0, 1.0)
            .addParameter("voxel_size", 0.02, 0.0, 1.0);
        return pattern;
    }

    void configureInstance(const pattern::instance::PatternInstance &pattern_instance) override {
        cameras_info_ = pattern_instance.getPortGroupInfo("camera");
    }

    virtual bool configure(const pattern::instance::PatternInstance &pattern_instance,
                           buffer::ComponentBufferConfig *data) override {
        pattern_instance.setValueFromParameter("reference_node", optimization_option_.reference_node_);
        pattern_instance.setValueFromParameter("edge_prune_threshold", optimization_option_.edge_prune_threshold_);
        pattern_instance.setValueFromParameter("preference_loop_closure", optimization_option_.preference_loop_closure_);

        pattern_instance.setValueFromParameter("voxel_size", voxel_size_);

        optimization_option_.max_correspondence_distance_ = voxel_size_ * 1.5;
        max_correspondence_distance_course_ = voxel_size_ * 15;

        // +1 for 0 is virtual origin node
        optimization_option_.reference_node_ += 1;


        return true;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {

        if (cameras_info_.size < 1) {
            SPDLOG_WARN("need at least 1 sub camera, error in dataflow setup");
            return true;
        }


        open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);

        point_cloud::MultiwayRegistrationConfig registration_config(cameras_info_.size, max_correspondence_distance_course_, optimization_option_.max_correspondence_distance_);


        for (auto camera_index = 0; camera_index < cameras_info_.size; ++camera_index) {
            point_cloud::downSample(data.getInput<InPortGroupCameraCloud>(cameras_info_.port_group_index, camera_index), registration_config.point_clouds[camera_index], voxel_size_);
            registration_config.origin_to_camera[camera_index] = data.getInput<InPortGroupCameraPose>(cameras_info_.port_group_index, camera_index).matrix().cast<double>();
        }

        for (auto source_id = 0; source_id < cameras_info_.size; ++source_id) {
            for (auto target_id = source_id+1; target_id < cameras_info_.size; ++target_id) {
                if(source_id == target_id) {
                    continue;
                }

                registration_config.edges.emplace_back(static_cast<size_t >(source_id), static_cast<size_t>(target_id), false);

            }

        }

        auto pose_graph = point_cloud::multiwayRegistration(registration_config);

        //auto clean_graph = open3d::pipelines::registration::CreatePoseGraphWithoutInvalidEdges(pose_graph, optimization_option_);


        open3d::pipelines::registration::GlobalOptimization(pose_graph, method_, convergence_criteria_, optimization_option_);

        for (auto camera_index = 0; camera_index < cameras_info_.size; ++camera_index) {
            data.getOutput<OutPortGroupCameraPose>(cameras_info_.port_group_index, camera_index).matrix() = pose_graph.nodes_[camera_index+1].pose_.cast<traact::Scalar>();
        }


        return true;
    }

    std::vector<int> createProcessOrder(buffer::ComponentBuffer &data) const {

        std::vector<std::pair<size_t, float>> sub_diff;
        sub_diff.reserve(cameras_info_.size);

        const auto &main_pose = data.getInput<InPortGroupCameraPose>(cameras_info_.port_group_index, optimization_option_.reference_node_);
        Eigen::Vector3f main_pos = main_pose.translation();

        for (int sub_index = 0; sub_index < cameras_info_.size; ++sub_index) {
            if(sub_index == optimization_option_.reference_node_){
                continue;
            }

            const auto &sub_pose = data.getInput<InPortGroupCameraPose>(cameras_info_.port_group_index, sub_index);
            Eigen::Vector3f sub_pos = sub_pose.translation();

            auto diff = std::atan2(main_pos.cross(sub_pos).norm(), main_pos.dot(sub_pos));
            sub_diff.emplace_back(sub_index, diff);
        }
        std::vector<int> process_order;
        std::sort(sub_diff.begin(), sub_diff.end(), [](const auto &value_1, const auto &value_2) {
            return value_1.second < value_2.second;
        });
        for (const auto &index_diff : sub_diff) {
            process_order.emplace_back(index_diff.first);

        }

        return process_order;
    }




 private:
    PortGroupInfo cameras_info_;
    open3d::pipelines::registration::GlobalOptimizationLevenbergMarquardt method_;
    open3d::pipelines::registration::GlobalOptimizationConvergenceCriteria convergence_criteria_;
    open3d::pipelines::registration::GlobalOptimizationOption optimization_option_;
    double max_correspondence_distance_course_;
    double voxel_size_;

};

CREATE_TRAACT_COMPONENT_FACTORY(Open3DMultiwayRegistration)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::Open3DMultiwayRegistration)
END_TRAACT_PLUGIN_REGISTRATION

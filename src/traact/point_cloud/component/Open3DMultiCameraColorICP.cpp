/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/vision.h>
#include "traact/point_cloud.h"

#include <open3d/Open3D.h>
#include "traact/point_cloud/open3d_algorithms.h"
namespace traact::component {

class Open3DMultiCameraColorICP : public Component {
 public:
    using OutPortFusedCloud = buffer::PortConfig<point_cloud::PointCloudHeader, 0>;

    using InPortGroupCameraPose = buffer::PortConfig<traact::spatial::Pose6DHeader, 0>;
    using InPortGroupCameraCloud = buffer::PortConfig<point_cloud::PointCloudHeader, 1>;
    using OutPortGroupCameraPose = buffer::PortConfig<traact::spatial::Pose6DHeader, 0>;

    explicit Open3DMultiCameraColorICP(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern =
            std::make_shared<traact::pattern::Pattern>("Open3DMultiCameraColorICP",
                                                       Concurrency::SERIAL,
                                                       ComponentType::SYNC_FUNCTIONAL);

        pattern->addProducerPort<OutPortFusedCloud>("output")
            .beginPortGroup("camera", 2)
            .addConsumerPort<InPortGroupCameraPose>("input_pose")
            .addConsumerPort<InPortGroupCameraCloud>("input_cloud")
            .addProducerPort<OutPortGroupCameraPose>("output")
            .endPortGroup()
            .addParameter("max_distance", 0.02, 0.0, 1.0)
            .addParameter("reference_node", 0, 0, 10);
        return pattern;
    }

    void configureInstance(const pattern::instance::PatternInstance &pattern_instance) override {
        cameras_info_ = pattern_instance.getPortGroupInfo("camera");
    }

    virtual bool configure(const pattern::instance::PatternInstance &pattern_instance,
                           buffer::ComponentBufferConfig *data) override {
        pattern_instance.setValueFromParameter("max_distance", max_distance_);
        pattern_instance.setValueFromParameter("reference_node", reference_node_);

        return true;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {

        if (cameras_info_.size < 2) {
            SPDLOG_WARN("need at least 2 cameras, error in dataflow setup");
            return true;
        }

        std::vector<bool> sub_processed(cameras_info_.size, false);

        const auto &main_pose = data.getInput<InPortGroupCameraPose>(cameras_info_.port_group_index, reference_node_);

        // copy point clouds to apply init transform
        // integrate all other sub clouds into it
        point_cloud::PointCloud main_cloud = std::make_shared<open3d::geometry::PointCloud>(*data.getInput<InPortGroupCameraCloud>(cameras_info_.port_group_index, reference_node_));

        main_cloud->Transform(main_pose.matrix().cast<double>());
        auto estimated_size = main_cloud->points_.size() * (cameras_info_.size + 1);
        main_cloud->points_.reserve(estimated_size);
        main_cloud->colors_.reserve(estimated_size);
        main_cloud->normals_.reserve(estimated_size);


        auto process_order = createProcessOrder(data);

        for (size_t order_index = 0; order_index < process_order.size(); ++order_index) {
            int sub_index = process_order.at(order_index);

            const auto
                &sub_pose_float = data.getInput<InPortGroupCameraPose>(cameras_info_.port_group_index, sub_index);
            auto sub_cloud = std::make_shared<open3d::geometry::PointCloud>(*data.getInput<InPortGroupCameraCloud>(cameras_info_.port_group_index, sub_index));

            Eigen::Matrix4d sub_pose = sub_pose_float.matrix().cast<double>();

            SPDLOG_INFO("color icp for sub camera index {0} ", sub_index);
            point_cloud::MultiStageColorIcpConfig config{main_cloud,
                                                         sub_cloud,
                                                         sub_pose,
                                                         false,
                                                         true,
                                                         Eigen::Vector3d()};


            Eigen::Matrix4d sub_pose_new = point_cloud::multiStageColorIcp(config);

            sub_cloud->Transform(sub_pose_new);

            for (auto i = 0; i < sub_cloud->points_.size(); ++i) {
                main_cloud->points_.emplace_back(sub_cloud->points_[i]);
                main_cloud->colors_.emplace_back(sub_cloud->colors_[i]);
                main_cloud->normals_.emplace_back(sub_cloud->normals_[i]);
            }
            data.getOutput<OutPortGroupCameraPose>(cameras_info_.port_group_index, sub_index) =
                spatial::Pose6D(sub_pose_new.cast<float>());

        }
        if(main_cloud){
            data.getOutput<OutPortFusedCloud>() = main_cloud;
        }

        return true;
    }

    std::vector<int> createProcessOrder(buffer::ComponentBuffer &data) const {

        std::vector<std::pair<size_t, float>> sub_diff;
        sub_diff.reserve(cameras_info_.size);

        const auto &main_pose = data.getInput<InPortGroupCameraPose>(cameras_info_.port_group_index, reference_node_);
        Eigen::Vector3f main_pos = main_pose.translation();

        for (int sub_index = 0; sub_index < cameras_info_.size; ++sub_index) {
            if(sub_index == reference_node_){
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
    double max_distance_;
    int reference_node_;

};

CREATE_TRAACT_COMPONENT_FACTORY(Open3DMultiCameraColorICP)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::Open3DMultiCameraColorICP)
END_TRAACT_PLUGIN_REGISTRATION

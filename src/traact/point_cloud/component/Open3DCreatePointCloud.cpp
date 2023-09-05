/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include <traact/vision.h>
#include "traact/point_cloud.h"
//#include <open3d/Open3D.h>
#include <open3d/camera/PinholeCameraIntrinsic.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/Image.h>
namespace traact::component {


class Open3DCreatePointCloud : public Component {
 public:
    using InDepthImage = buffer::PortConfig<traact::vision::ImageHeader, 0>;
    using InDepthCalibrationImage = buffer::PortConfig<traact::vision::CameraCalibrationHeader, 1>;

    using OutPointCloud = buffer::PortConfig<traact::point_cloud::PointCloudHeader, 0>;

    explicit Open3DCreatePointCloud(const std::string &name) : Component(name) {}

    static pattern::Pattern::Ptr GetPattern(){

        pattern::Pattern::Ptr
            pattern = std::make_shared<pattern::Pattern>("Open3DCreatePointCloud", traact::Concurrency::SERIAL, ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InDepthImage>("input_depth_image")
            .addConsumerPort<InDepthCalibrationImage>("input_depth_calibration")
            .addProducerPort<OutPointCloud>("output");



        return
            pattern;
    };

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        const auto &depth_image = data.getInput<InDepthImage>().value();
        const auto &depth_calibration = data.getInput<InDepthCalibrationImage>();

        auto &output = data.getOutput<OutPointCloud>();

        open3d::camera::PinholeCameraIntrinsic intrinsic(depth_calibration.width, depth_calibration.height, depth_calibration.fx, depth_calibration.fy, depth_calibration.cx, depth_calibration.cy);

        open3d::geometry::Image image;
        image.Prepare(depth_calibration.width, depth_calibration.height, 1, 2);

        memcpy(image.PointerAs<uint8_t>(), depth_image.data, depth_calibration.width*depth_calibration.height*2);

        output = open3d::geometry::PointCloud::CreateFromDepthImage(image, intrinsic, Eigen::Matrix4d::Identity(), 1000.0, 10000.0);

        return true;
    }


};
CREATE_TRAACT_COMPONENT_FACTORY(Open3DCreatePointCloud)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::Open3DCreatePointCloud)
END_TRAACT_PLUGIN_REGISTRATION

/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include <traact/vision.h>
#include <traact/component/CudaComponent.h>
#include "CudaComponentFunctions.cuh"

namespace traact::component {

class CudaColorPointCloud : public CudaComponent {
 public:
    using InPortPointCloud = buffer::PortConfig<vision::GpuImageHeader, 0>;
    using InPortColorImage = buffer::PortConfig<vision::GpuImageHeader, 1>;
    using InPortColorCalibration = buffer::PortConfig<vision::CameraCalibrationHeader, 2>;
    using InPortColorToDepth = buffer::PortConfig<spatial::Pose6DHeader, 3>;

    using OutPortPointCloudColor = buffer::PortConfig<vision::GpuImageHeader, 0>;

    explicit CudaColorPointCloud(const std::string &name) : CudaComponent(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern = CudaComponent::GetPattern("CudaColorPointCloud",
                                                Concurrency::SERIAL,
                                                ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPortPointCloud>("input")
            .addConsumerPort<InPortColorImage>("input_color")
            .addConsumerPort<InPortColorCalibration>("input_color_calibration")
            .addConsumerPort<InPortColorToDepth>("input_color_to_depth")
            .addProducerPort<OutPortPointCloudColor>("output")
            .addParameter("depthScale", 1000.0, 0.0, 10000.0);

        return pattern;
    }

    virtual bool configure(const pattern::instance::PatternInstance &pattern_instance,
                           buffer::ComponentBufferConfig *data) override {

        return true;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        using namespace traact::vision;
        const auto &image = data.getInput<InPortPointCloud>().value();

        auto &output = data.getOutput<OutPortPointCloudColor>().value();

        output.create(image.rows, image.cols, CV_8UC4);

        auto &output_header = data.getOutputHeader<OutPortPointCloudColor>();
        output_header.setFrom(output);
        output_header.pixel_format = PixelFormat::RGBA;

        return true;
    }

    CudaTask createGpuTask(buffer::ComponentBuffer *data) override {
        return [data](cudaStream_t stream) {
            if (data->isInputValid<InPortPointCloud>() &&
                data->isInputValid<InPortColorImage>() &&
                data->isInputValid<InPortColorCalibration>() &&
                data->isInputValid<InPortColorToDepth>()) {
                const auto &point_cloud = data->getInput<InPortPointCloud>().value();
                const auto &color_image = data->getInput<InPortColorImage>().value();
                const auto &calibration = data->getInput<InPortColorCalibration>();
                const auto &color_to_depth = data->getInput<InPortColorToDepth>();

                auto &output = data->getOutput<OutPortPointCloudColor>().value();

                vision::colorPointCloud(point_cloud, color_image, calibration, color_to_depth, output, stream);
            }

        };
    }

 private:
};

CREATE_TRAACT_COMPONENT_FACTORY(CudaColorPointCloud)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::CudaColorPointCloud)
END_TRAACT_PLUGIN_REGISTRATION
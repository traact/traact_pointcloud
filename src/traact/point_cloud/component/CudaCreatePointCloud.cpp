/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include <traact/vision.h>
#include <traact/component/CudaComponent.h>
#include "CudaComponentFunctions.cuh"

namespace traact::component {

class CudaCreatePointCloud : public CudaComponent {
 public:
    using InPortDepthImage = buffer::PortConfig<vision::GpuImageHeader, 0>;
    using InPortXYTable = buffer::PortConfig<vision::GpuImageHeader, 1>;

    using OutPortPointCloud = buffer::PortConfig<vision::GpuImageHeader, 0>;

    explicit CudaCreatePointCloud(const std::string &name) : CudaComponent(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern = CudaComponent::GetPattern("CudaCreatePointCloud",
                                                Concurrency::SERIAL,
                                                ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPortDepthImage>("input")
            .addConsumerPort<InPortXYTable>("input_xy_table")
            .addProducerPort<OutPortPointCloud>("output")
            .addParameter("depthScale", 1000.0, 0.0, 10000.0);

        return pattern;
    }

    virtual bool configure(const pattern::instance::PatternInstance &pattern_instance,
                           buffer::ComponentBufferConfig *data) override {

        return true;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        using namespace traact::vision;
        const auto &image = data.getInput<InPortDepthImage>().value();
        const auto &calibration = data.getInput<InPortXYTable>();

        auto &output = data.getOutput<OutPortPointCloud>().value();

        output.create(image.rows, image.cols, CV_32FC4);

        auto &output_header = data.getOutputHeader<OutPortPointCloud>();
        output_header.setFrom(output);
        output_header.pixel_format = PixelFormat::FLOAT;

        return true;
    }

    CudaTask createGpuTask(buffer::ComponentBuffer *data) override {
        return [data, this](cudaStream_t stream) {
            if (data->isInputValid<InPortDepthImage>() && data->isInputValid<InPortXYTable>()) {
                const auto &depth_image = data->getInput<InPortDepthImage>().value();
                const auto &xy_table = data->getInput<InPortXYTable>().value();
                auto &output = data->getOutput<OutPortPointCloud>().value();

                vision::createPointCloud(depth_image, xy_table, depth_scale_, output, stream);
            }

        };
    }

 private:
    float depth_scale_{1000.0};

};

CREATE_TRAACT_COMPONENT_FACTORY(CudaCreatePointCloud)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::CudaCreatePointCloud)
END_TRAACT_PLUGIN_REGISTRATION
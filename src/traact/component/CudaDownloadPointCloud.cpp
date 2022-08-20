/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include <traact/vision.h>
#include <traact/pointCloud.h>
#include <traact/component/CudaComponent.h>
#include "CudaComponentFunctions.cuh"

namespace traact::component {

class CudaDownloadPointCloud : public CudaComponent {
 public:
    using InPortPointCloud = buffer::PortConfig<vision::GpuImageHeader, 0>;
    

    using OutPortPointCloud = buffer::PortConfig<pointCloud::PointCloudHeader, 0>;

    explicit CudaDownloadPointCloud(const std::string &name) : CudaComponent(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern = CudaComponent::GetPattern("CudaDownloadPointCloud",
                                                Concurrency::SERIAL,
                                                ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPortPointCloud>("input")
            .addProducerPort<OutPortPointCloud>("output");

        return pattern;
    }

    virtual bool configure(const pattern::instance::PatternInstance &pattern_instance,
                           buffer::ComponentBufferConfig *data) override {

        return true;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        using namespace traact::vision;
        const auto &input = data.getInput<InPortPointCloud>().value();


        auto &output = data.getOutput<OutPortPointCloud>();

        if(!output){
            output.reset(new open3d::geometry::PointCloud());
        }

        output->points_.resize(input.rows*input.cols);

        return true;
    }

    CudaTask createGpuTask(buffer::ComponentBuffer *data) override {
        return [data, this](cudaStream_t stream) {
            if (data->isInputValid<InPortPointCloud>() ) {
                const auto &input = data->getInput<InPortPointCloud>().value();
                auto &output = data->getOutput<OutPortPointCloud>();

                size_t count = output->points_.size() * sizeof(double)*3;
                cudaMemcpyAsync(input.cudaPtr(), output->points_.data(), count, cudaMemcpyKind::cudaMemcpyDeviceToHost, stream);

            }

        };
    }

 private:
    float depth_scale_{1000.0};

};

CREATE_TRAACT_COMPONENT_FACTORY(CudaDownloadPointCloud)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::CudaDownloadPointCloud)
END_TRAACT_PLUGIN_REGISTRATION
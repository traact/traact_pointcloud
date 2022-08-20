/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include <traact/vision.h>
#include <traact/pointCloud.h>
#include <traact/component/CudaComponent.h>
#include <open3d/io/PointCloudIO.h>
#include "CudaComponentFunctions.cuh"

namespace traact::component {

class Open3dBuildPointCloud : public Component {
 public:
    using InPortPointCloud = buffer::PortConfig<vision::ImageHeader, 0>;
    using InPortPointColor = buffer::PortConfig<vision::ImageHeader, 1>;


    using OutPortPointCloud = buffer::PortConfig<pointCloud::PointCloudHeader, 0>;

    explicit Open3dBuildPointCloud(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern = CudaComponent::GetPattern("Open3dBuildPointCloud",
                                                Concurrency::SERIAL,
                                                ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPortPointCloud>("input")
            .addConsumerPort<InPortPointColor>("input_color")
            .addProducerPort<OutPortPointCloud>("output")
                .addParameter("minDistance", 0.1, 0.0, 10.0)
                .addParameter("maxDistance", 10.0, 0.0, 10.0);

        return pattern;
    }

    virtual bool configure(const pattern::instance::PatternInstance &pattern_instance,
                           buffer::ComponentBufferConfig *data) override {
        pattern_instance.setValueFromParameter("minDistance", min_distance_);
        pattern_instance.setValueFromParameter("maxDistance", max_distance_);

        return true;
    }

    bool processTimePoint(buffer::ComponentBuffer &data) override {
        using namespace traact::vision;
        const auto &input = data.getInput<InPortPointCloud>().value();
        const auto &input_color = data.getInput<InPortPointColor>().value();


        auto &output = data.getOutput<OutPortPointCloud>();

        if(!output){
            output.reset(new open3d::geometry::PointCloud());
        }

        output->Clear();
        output->points_.reserve(input.rows*input.cols);
        output->colors_.reserve(input.rows*input.cols);

        for(int r=0; r<input.rows; ++r)
        {
            for(int c=0; c<input.cols; ++c)
            {
                auto point = input.at<cv::Vec4f>(r,c);
                auto color = input_color.at<cv::Vec4b>(r,c);
                if(point[2] > min_distance_ && point[2] < max_distance_){
                    output->points_.emplace_back(point[0],point[1],point[2]);
                    output->colors_.emplace_back(color[0]/255.0,color[1]/255.0,color[2]/255.0);
                }

            }
        }

//        static bool written{false};
//
//        if (!written) {
//            open3d::io::WritePointCloudOption option;
//
//            open3d::io::WritePointCloud("test.xyzrgb", *output);
//            open3d::io::WritePointCloud("test.ply", *output);
//            spdlog::error("written file");
//            written = true;
//        }


        return true;
    }



 private:
    float min_distance_{0.1};
    float max_distance_{10.0};


};

CREATE_TRAACT_COMPONENT_FACTORY(Open3dBuildPointCloud)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::Open3dBuildPointCloud)
END_TRAACT_PLUGIN_REGISTRATION
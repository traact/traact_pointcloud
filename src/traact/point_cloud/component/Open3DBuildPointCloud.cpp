/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include <traact/vision.h>
#include "traact/point_cloud.h"
#include <traact/component/CudaComponent.h>
#include <open3d/io/PointCloudIO.h>
#include "CudaComponentFunctions.cuh"
#include "traact/point_cloud/open3d_algorithms.h"
namespace traact::component {

class Open3DBuildPointCloud : public Component {
 public:
    using InPortPointCloud = buffer::PortConfig<vision::ImageHeader, 0>;
    using InPortPointColor = buffer::PortConfig<vision::ImageHeader, 1>;


    using OutPortPointCloud = buffer::PortConfig<point_cloud::PointCloudHeader, 0>;

    explicit Open3DBuildPointCloud(const std::string &name) : Component(name) {
    }

    static traact::pattern::Pattern::Ptr GetPattern() {

        traact::pattern::Pattern::Ptr
            pattern = CudaComponent::GetPattern("Open3DBuildPointCloud",
                                                Concurrency::SERIAL,
                                                ComponentType::SYNC_FUNCTIONAL);

        pattern->addConsumerPort<InPortPointCloud>("input")
            .addConsumerPort<InPortPointColor>("input_color")
            .addProducerPort<OutPortPointCloud>("output")
                .addParameter("min_distance", 0.1, 0.0, 10.0)
                .addParameter("max_distance", 10.0, 0.0, 10.0)
                .addParameter("estimate_normals", true, true, false)
                .addParameter("estimate_normals_radius", 0.01, 0.0, 1.0);


        return pattern;
    }

    virtual bool configure(const pattern::instance::PatternInstance &pattern_instance,
                           buffer::ComponentBufferConfig *data) override {
        pattern_instance.setValueFromParameter("min_distance", min_distance_);
        pattern_instance.setValueFromParameter("max_distance", max_distance_);
        pattern_instance.setValueFromParameter("estimate_normals", estimate_normals_);
        pattern_instance.setValueFromParameter("estimate_normals_radius", estimate_normals_radius_);

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

        if(estimate_normals_){
            point_cloud::estimateNormals(output, estimate_normals_radius_, Eigen::Vector3d());
        }


        return true;
    }



 private:
    float min_distance_{0.1};
    float max_distance_{10.0};
    bool estimate_normals_{true};
    double estimate_normals_radius_{0.01};


};

CREATE_TRAACT_COMPONENT_FACTORY(Open3DBuildPointCloud)

}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::component::Open3DBuildPointCloud)
END_TRAACT_PLUGIN_REGISTRATION
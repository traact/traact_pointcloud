/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACT_COMPONENT_POINTCLOUD_SRC_TRAACT_POINTCLOUD_H_
#define TRAACT_COMPONENT_POINTCLOUD_SRC_TRAACT_POINTCLOUD_H_



#include <traact/traact.h>
#include <open3d/geometry/PointCloud.h>
//#include <traact/traact_component_pointcloud_export.h>

#define TRAACT_COMPONENT_POINTCLOUD_EXPORT

namespace traact::pointCloud {

using PointCloud = std::shared_ptr<open3d::geometry::PointCloud>;

CREATE_TRAACT_HEADER_TYPE(PointCloudHeader, traact::pointCloud::PointCloud, "pointCloud:PointCloud", TRAACT_COMPONENT_POINTCLOUD_EXPORT)



}// namespace traact::spatial

#define CREATE_POINT_CLOUD_COMPONENTS(external_component) \
CREATE_TEMPLATED_TRAACT_COMPONENT_FACTORY(external_component, traact::pointCloud, PointCloudHeader)


#define REGISTER_POINT_CLOUD_COMPONENTS(external_component) \
REGISTER_TEMPLATED_DEFAULT_COMPONENT(external_component, PointCloudHeader)

#endif //TRAACT_COMPONENT_POINTCLOUD_SRC_TRAACT_POINTCLOUD_H_

/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef traact_pointcloud_SRC_TRAACT_POINT_CLOUD_H_
#define traact_pointcloud_SRC_TRAACT_POINT_CLOUD_H_



#include <traact/traact.h>
#include "point_cloud/point_cloud_datatypes.h"
#include "traact/traact_pointcloud_export.h"

namespace traact::point_cloud {



CREATE_TRAACT_HEADER_TYPE(PointCloudHeader, traact::point_cloud::PointCloud, "pointCloud:PointCloud", TRAACT_POINTCLOUD_EXPORT)



}// namespace traact::spatial

#define CREATE_POINT_CLOUD_COMPONENTS(external_component) \
CREATE_TEMPLATED_TRAACT_COMPONENT_FACTORY(external_component, traact::point_cloud, PointCloudHeader)


#define REGISTER_POINT_CLOUD_COMPONENTS(external_component) \
REGISTER_TEMPLATED_DEFAULT_COMPONENT(external_component, PointCloudHeader)

#endif //traact_pointcloud_SRC_TRAACT_POINT_CLOUD_H_

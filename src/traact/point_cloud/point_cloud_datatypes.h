/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACT_COMPONENT_POINTCLOUD_SRC_TRAACT_POINT_CLOUD_POINT_CLOUD_DATATYPES_H_
#define TRAACT_COMPONENT_POINTCLOUD_SRC_TRAACT_POINT_CLOUD_POINT_CLOUD_DATATYPES_H_

#include <open3d/geometry/PointCloud.h>

namespace traact::point_cloud {
using PointCloud = std::shared_ptr<open3d::geometry::PointCloud>;
}
#endif //TRAACT_COMPONENT_POINTCLOUD_SRC_TRAACT_POINT_CLOUD_POINT_CLOUD_DATATYPES_H_

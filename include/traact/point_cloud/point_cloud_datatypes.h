/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef traact_pointcloud_SRC_TRAACT_POINT_CLOUD_POINT_CLOUD_DATATYPES_H_
#define traact_pointcloud_SRC_TRAACT_POINT_CLOUD_POINT_CLOUD_DATATYPES_H_

#include <open3d/geometry/PointCloud.h>

namespace traact::point_cloud {
using PointCloud = std::shared_ptr<open3d::geometry::PointCloud>;
}
#endif //traact_pointcloud_SRC_TRAACT_POINT_CLOUD_POINT_CLOUD_DATATYPES_H_

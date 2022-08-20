/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "pointCloud.h"

#include <traact/component/generic_components.h>

namespace  traact::component::facade {
CREATE_POINT_CLOUD_COMPONENTS(ApplicationAsyncSource)
CREATE_POINT_CLOUD_COMPONENTS(ApplicationSyncSink)
}

namespace  traact::component {
CREATE_POINT_CLOUD_COMPONENTS(Buffer)
CREATE_POINT_CLOUD_COMPONENTS(Gate)
}

BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_TRAACT_TYPE(traact::pointCloud::PointCloudHeader)
    REGISTER_POINT_CLOUD_COMPONENTS(traact::component::facade::ApplicationAsyncSource)
    REGISTER_POINT_CLOUD_COMPONENTS(traact::component::facade::ApplicationSyncSink)
    REGISTER_POINT_CLOUD_COMPONENTS(traact::component::Buffer)
    REGISTER_POINT_CLOUD_COMPONENTS(traact::component::Gate)
END_TRAACT_PLUGIN_REGISTRATION

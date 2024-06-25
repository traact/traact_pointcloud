cmake_minimum_required(VERSION 3.16)

set(TARGET_NAME traact_pointcloud)

project(${TARGET_NAME} VERSION "0.0.0" LANGUAGES CXX)

find_package(traact_base)
list(APPEND CMAKE_MODULE_PATH ${traact_base_INCLUDE_DIR})

include(traact_default_library_setup)

# add your source files here
set(PULIC_HEADER 
    include/traact/point_cloud.h
    include/traact/point_cloud/open3d_algorithms.h 
    include/traact/point_cloud/point_cloud_datatypes.h)

set(COMPONENT_SRC src/traact/point_cloud/component/Open3DCreatePointCloud.cpp  src/traact/point_cloud.cpp src/traact/point_cloud/component/CudaCreatePointCloud.cpp src/traact/point_cloud/component/CudaComponentFunctions.cu src/traact/point_cloud/component/CudaComponentFunctions.cuh src/traact/point_cloud/component/CudaDownloadPointCloud.cpp src/traact/point_cloud/component/Open3DBuildPointCloud.cpp src/traact/point_cloud/component/CudaColorPointCloud.cpp src/traact/point_cloud/component/Open3DMultiCameraColorICP.cpp src/traact/point_cloud/component/Open3DMultiwayRegistration.cpp src/traact/point_cloud/open3d_algorithms.cpp )

add_library(${TARGET_NAME} SHARED ${COMPONENT_SRC})

find_package(traact_spatial REQUIRED)
find_package(traact_vision REQUIRED)
find_package(open3d REQUIRED)

target_link_libraries(${TARGET_NAME} traact_core::traact_core traact_spatial::traact_spatial traact_vision::traact_vision open3d::open3d)

generate_export_header(${TARGET_NAME} EXPORT_FILE_NAME ${CMAKE_CURRENT_SOURCE_DIR}/include/traact/traact_pointcloud_export.h)

target_include_directories(${TARGET_NAME} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

include(traact_default_library_final)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
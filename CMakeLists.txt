cmake_minimum_required(VERSION 3.16)

if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
    include(${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
else ()
    message(FATAL_ERROR "The file conanbuildinfo.cmake doesn't exist, you have to run conan install first")
endif ()

list(APPEND CMAKE_MODULE_PATH ${CONAN_TRAACT_RUN_ENV_ROOT}/cmake)

project(${CONAN_PACKAGE_NAME}_PROJECT VERSION ${CONAN_PACKAGE_VERSION})

include(traact_default_library_setup)

# add your source files here
set(COMPONENT_SRC src/traact/point_cloud/component/Open3DCreatePointCloud.cpp src/traact/point_cloud.h src/traact/point_cloud.cpp src/traact/point_cloud/component/CudaCreatePointCloud.cpp src/traact/point_cloud/component/CudaComponentFunctions.cu src/traact/point_cloud/component/CudaComponentFunctions.cuh src/traact/point_cloud/component/CudaDownloadPointCloud.cpp src/traact/point_cloud/component/Open3DBuildPointCloud.cpp src/traact/point_cloud/component/CudaColorPointCloud.cpp src/traact/point_cloud/component/Open3DMultiCameraColorICP.cpp src/traact/point_cloud/component/Open3DMultiwayRegistration.cpp src/traact/point_cloud/open3d_algorithms.cpp src/traact/point_cloud/open3d_algorithms.h src/traact/point_cloud/point_cloud_datatypes.h)


add_library(${CONAN_PACKAGE_NAME} SHARED ${COMPONENT_SRC})

conan_target_link_libraries(${CONAN_PACKAGE_NAME})

install(DIRECTORY "src/traact" # source directory
        DESTINATION "include" # target directory
        FILES_MATCHING # install only matched files
        PATTERN "*.h" # select header files
        )

include(traact_default_library_final)
# /usr/bin/python3
import os
from conans import ConanFile, CMake, tools


class TraactPackage(ConanFile):
    python_requires = "traact_run_env/1.0.0@traact/latest"
    python_requires_extend = "traact_run_env.TraactPackageCmake"

    name = "Point Cloud Datatypes and Components based on Open3D"
    description = "..."
    url = "https://github.com/traact/traact_component_pointcloud.git"
    license = "MIT"
    author = "Frieder Pankratz"

    settings = "os", "compiler", "build_type", "arch"
    compiler = "cppstd"

    def _options(self):
        pass

    exports_sources = "src/*", "CMakeLists.txt"

    # overwrite these dependencies
    requires = (
        "eigen/3.4.0"
    )

    def requirements(self):
        # add your dependencies
        self.traact_requires("traact_vision", "latest")
        self.traact_requires("traact_spatial", "latest")
        self.requires("open3d/0.16.1@camposs/stable")
        #self.requires("fmt/9.1.0")

    def configure(self):
        #self.options['open3d'].with_visualization = True
        #self.options['open3d'].with_kinect = True
        pass


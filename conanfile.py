# /usr/bin/python3
import os
from conan import ConanFile
from conan.tools.build import can_run

class TraactPackage(ConanFile):
    python_requires = "traact_base/0.0.0@traact/latest"
    python_requires_extend = "traact_base.TraactPackageCmake"

    name = "traact_pointcloud"
    version = "0.0.0"
    description = "Point Cloud Datatypes and Components based on Open3D"
    url = "https://github.com/traact/traact_pointcloud.git"
    license = "MIT"
    author = "Frieder Pankratz"

    settings = "os", "compiler", "build_type", "arch"
    compiler = "cppstd"
    
    exports_sources = "src/*", "include/*", "CMakeLists.txt"    

    options = {
        "shared": [True, False],
        "trace_logs_in_release": [True, False]
    }

    default_options = {
        "shared": True,
        "trace_logs_in_release": True
    }

    def requirements(self):
        # add your dependencies
        self.requires("traact_vision/0.0.0@traact/latest")
        self.requires("traact_spatial/0.0.0@traact/latest")
        self.requires("open3d/0.17.0@camposs/stable", transitive_headers=True, transitive_libs=True)  
              

    def configure(self):
        #self.options['open3d'].with_visualization = True
        #self.options['open3d'].with_kinect = True
        pass

    def _after_package_info(self):        
        self.cpp_info.libs = ["traact_pointcloud"]


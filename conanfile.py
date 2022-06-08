# /usr/bin/python3
import os
from conans import ConanFile, CMake, tools


class TraactPackage(ConanFile):
    python_requires = "traact_run_env/1.0.0@traact/latest"
    python_requires_extend = "traact_run_env.TraactPackageCmake"

    name = "your_component_name"
    description = "..."
    url = ""
    license = ""
    author = ""

    settings = "os", "compiler", "build_type", "arch"
    compiler = "cppstd"

    def _options(self):
        self.options["with_bodytracking"] = [True, False]
        self.default_options["with_bodytracking"] = True

    exports_sources = "src/*", "CMakeLists.txt"

    def requirements(self):
        # add your dependencies
        self.traact_requires("traact_vision", "latest")
        if self.options.with_tests:
            self.requires("gtest/[>=1.11.0]")
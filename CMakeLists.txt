cmake_minimum_required(VERSION 3.20 FATAL_ERROR)

project(ros-tidy)

find_package(LLVM REQUIRED CONFIG)
find_package(Clang REQUIRED CONFIG)

add_library(RosTidyModule MODULE "")

target_include_directories(RosTidyModule
    PRIVATE
        ${CLANG_INCLUDE_DIRS}
        ${LLVM_INCLUDE_DIRS}
)

target_sources(RosTidyModule
    PRIVATE
        src/ros-tidy/RosTidyModule.cpp
        src/ros-tidy/RosInterfaceNamesCheck.cpp
)

target_include_directories(RosTidyModule PUBLIC include)
project(3DKeypointGeneration)
cmake_minimum_required(VERSION 2.8)
FIND_PACKAGE(OpenCV REQUIRED)
SET(CMAKE_BUILD_TYPE release)
ADD_EXECUTABLE(3DKeypointGeneration fileoperation.cpp featureoperation.cpp main.cpp rply.c)
TARGET_LINK_LIBRARIES(3DKeypointGeneration ${OpenCV_LIBS} siftgpu boost_system boost_program_options boost_filesystem)


cmake_minimum_required(VERSION 2.8)
project(sift_for_bundler)
FIND_PACKAGE(OpenCV REQUIRED)
add_executable(sift_for_bundler main.cpp)
TARGET_LINK_LIBRARIES(sift_for_bundler ${OpenCV_LIBS} siftgpu)


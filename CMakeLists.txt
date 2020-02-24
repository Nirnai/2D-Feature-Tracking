cmake_minimum_required(VERSION 3.4)
project(CameraFusion LANGUAGES CXX)

find_package(OpenCV 4.2 REQUIRED)

add_executable(2D_feature_tracking src/matching2D_Student.cpp src/MidTermProject_Camera_Student.cpp)
target_include_directories(2D_feature_tracking PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(2D_feature_tracking PUBLIC ${OpenCV_LIBRARIES})
target_compile_features(2D_feature_tracking PUBLIC cxx_std_11)

cmake_minimum_required(VERSION 2.8.11)
project(cam_calibration)

add_executable(${PROJECT_NAME}
 cam_calibration.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
 ${XP_INCLUDE_DIR}
 ${TagDetector_INCLUDE_DIR}
 ${Eigen_INCLUDE_DIR}
 ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
 ${OpenCV_LIBRARIES}
 ${GLOG_LIBRARY}
 ${GFLAGS_LIBRARY}
 ${Boost_LIBRARIES}
 ${XP_LIBRARY}
)

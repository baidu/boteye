cmake_minimum_required(VERSION 2.8.11)
# ----------------------------
# Multithread logging
# ----------------------------
project(xp_sensor_logger)

add_executable(${PROJECT_NAME}
 xp_sensor_logger.cpp
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
 ${XP_LIBRARY}
 ${Boost_LIBRARIES}
)

cmake_minimum_required (VERSION 2.8)

project (KalmanFilter)

FIND_PACKAGE(OpenCV 4 REQUIRED)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

aux_source_directory(. DIR_SRCS)

add_executable(${PROJECT_NAME} ${DIR_SRCS})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS})
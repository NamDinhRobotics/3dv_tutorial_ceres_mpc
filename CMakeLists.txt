cmake_minimum_required(VERSION 3.2)

project(3dv_tutorial)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
#find Eigen3
find_package(Eigen3 REQUIRED)
message(STATUS "  Eigen3 include path: ${EIGEN3_INCLUDE_DIR}")

set(SRC_DIR	"${CMAKE_SOURCE_DIR}/examples")
set(BIN_DIR	"${CMAKE_SOURCE_DIR}/bin")

file(GLOB APP_SOURCES "${SRC_DIR}/*.cpp")

message(STATUS "  CERES  include path: ${CERES_INCLUDE_DIRS}")
message(STATUS "  CERES  library path: ${CERES_LIBRARIES}")

message(STATUS "  APP_SOURCES  library path: ${APP_SOURCES}")

#include_directories(${CERES_INCLUDE_DIRS})
message("ok------------------")

foreach(app_source ${APP_SOURCES})
    string(REPLACE ".cpp" "" app_name ${app_source})
    string(REPLACE "${SRC_DIR}/" "" app_name ${app_name})
    add_executable(${app_name} ${app_source})
    target_include_directories(${app_name} PUBLIC ${CERES_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(${app_name} ${OpenCV_LIBS} ${CERES_LIBRARIES})
    install(TARGETS ${app_name} DESTINATION ${BIN_DIR})
endforeach(app_source ${APP_SOURCES})

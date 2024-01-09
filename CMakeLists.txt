cmake_minimum_required(VERSION 3.5)
project(cmake_template)

message("start make cmake_template")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

set(GLOG_INCLUDE_DIRS "/usr/include/glog/")
set(GLOG_LIBRARIES "/usr/lib/x86_64-linux-gnu/libglog.so")

set(GFLAGS_INCLUDE_DIRS "/usr/include/gflags/")
set(GFLAGS_LIBRARIES "/usr/lib/x86_64-linux-gnu/libgflags.so")


set(ALL_TARGET_LIBRARIES "")
include(cmake/boost.cmake)
include(cmake/global_defination.cmake)
include(cmake/eigen.cmake)
include(cmake/OpenCV.cmake)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIRS}
        ${GFLAGS_INCLUDE_DIRS}
)

file(GLOB_RECURSE ALL_SRCS "src/*.cpp")

add_executable(pnp_benchmark node/pnp_benchmark.cpp ${ALL_SRCS})
target_link_libraries(pnp_benchmark ${ALL_TARGET_LIBRARIES} ${GLOG_LIBRARIES} ${GFLAGS_LIBRARIES})


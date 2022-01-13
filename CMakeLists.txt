cmake_minimum_required(VERSION 3.5)
project(cmake_template)

message("start make cmake_template")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
set(CMAKE_CXX_STANDARD 17)

set(ALL_TARGET_LIBRARIES "")
include(cmake/boost.cmake)
include(cmake/global_defination.cmake)
#include(cmake/geographic.cmake)
include(cmake/glog.cmake)
include(cmake/PCL.cmake)


find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE ALL_SRCS "src/*.cpp")

add_executable(template_node node/template_node.cpp ${ALL_SRCS})
target_link_libraries(template_node ${ALL_TARGET_LIBRARIES} ${Sophus_LIBRARIES})


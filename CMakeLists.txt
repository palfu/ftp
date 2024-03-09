if(DEFINED ENV{DEVOPS_ROOT_DIR})
else()
cmake_minimum_required(VERSION 3.0)
project(ftp)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
# add_definitions(-w)
add_definitions(-DARCH_X86)
add_definitions(-D__LINUX__)

option(BUILD_WITH_GLOG "build with glog" ON)
add_definitions(-DBUILD_WITH_GLOG=1)
# set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -std=c++14 -O3 -g -Wall -fPIC -fopenmp -pthread")
# set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -std=c++14 -O3 -Wall -fPIC -fopenmp -pthread")
set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -std=c++14 -O3 -g -fPIC -fopenmp -pthread")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -std=c++14 -O3 -fPIC -fopenmp -pthread")


include(cmake/sys_arch.cmake)
include(cmake/find_dependencies.cmake)
include(cmake/Protobuf.cmake)

if(NOT DEFINED DEPENDENCIES_PATH)
    if(EXISTS /dependencies)
        set(DEPENDENCIES_PATH /dependencies/)
    else()
        set(DEPENDENCIES_PATH ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/)
    endif()

    message(STATUS "DEPENDENCIES_PATH not set, use default ${DEPENDENCIES_PATH}")
    message(STATUS "cmake ../  -DDEPENDENCIES_PATH=xxx")
endif()
message(STATUS ${DEPENDENCIES_PATH})
set_dependencies_dir(${DEPENDENCIES_PATH})

#****************************************************************#
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

find_dependency_thirdparty(jsoncpp)
find_dependency_thirdparty(eigen3)
find_dependency_thirdparty(glog)
find_dependency_thirdparty(gflags)

include_directories(
    ${jsoncpp_INCLUDE_DIRS}
    ${boost_INCLUDE_DIRS}
    ${glog_INCLUDE_DIRS}
    ${gflags_INCLUDE_DIRS}
)

link_directories(
    ${jsoncpp_LIB_DIRS}
    ${glog_LIB_DIRS}
    ${gflags_LIB_DIRS}
)
message("easy ${easy_profiler_LIB_DIRS}")


set(BASIC_LIBRARIES  ${boost_LIBS}  ${jsoncpp_LIBS} ${glog_LIBS} ${gflags_LIBS})

# add_subdirectory(filter)
add_subdirectory(example)

configure_file(cmake/run_env.sh.in ${CMAKE_CURRENT_BINARY_DIR}/env.sh @ONLY)

# Add a custom target to run the script
add_custom_target(run_env COMMAND ${CMAKE_CURRENT_BINARY_DIR}/env.sh)

endif()

cmake_minimum_required(VERSION 2.8.3)
project(apriltag3_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -DEIGEN_NO_DEBUG -march=native -Wl,--no-as-needed")
set(APRILTAG3_SRC_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(APRILTAG3_INCLUDE_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(APRILTAG3_DIR ${CMAKE_CURRENT_SOURCE_DIR}/apriltag3)
set(APRILTAG3_COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/apriltag3/common)

message("-->APRILTAG3_SRC_FOLDER: ${APRILTAG3_SRC_FOLDER}")
message("-->APRILTAG3_INCLUDE_FOLDER: ${APRILTAG3_INCLUDE_FOLDER}")

# Get the information about this package's buildtime dependencies
find_package(catkin REQUIRED COMPONENTS
    image_transport
    roscpp
    sensor_msgs
    geometry_msgs
    tf
    cv_bridge
#     message_generation
    nodelet
    std_msgs
    cmake_modules
    message_filters
    apriltag_msgs
    )

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED
    COMPONENTS filesystem system signals regex date_time program_options thread)

# Declare the message files to be built
# add_message_files(
#     FILES
#     AprilTagDetection.msg
#     AprilTagDetectionArray.msg
#     )
# 
# # Actually generate the language-specific message and service files
# generate_messages(
#     DEPENDENCIES
#     std_msgs
#     geometry_msgs
#     )

# Declare that this catkin package's runtime dependencies
catkin_package(
    INCLUDE_DIRS
    #include

    LIBRARIES

    CATKIN_DEPENDS
    image_transport
    roscpp
    sensor_msgs
    geometry_msgs
    message_filters
    tf
    cv_bridge
    #message_runtime
    nodelet
    std_msgs

    DEPENDS
    Eigen3
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    SYSTEM ${EIGEN3_INCLUDE_DIRS}
    ${APRILTAG3_INCLUDE_FOLDER}
    ${APRILTAG3_DIR}
    ${APRILTAG3_COMMON_DIR}
    )

# Make sure Boost.Filesystem v3 is used.
message("-->Verifying BOOST version...")
add_definitions(-DBOOST_FILESYSTEM_VERSION=3)
message("-->Boost_INCLUDE_DIRS:${Boost_INCLUDE_DIRS}")
message("-->...done")


# set( CMAKE_BUILD_TYPE Release)
# set( CMAKE_BUILD_DIRECTORY "${APRILTAG3_DIR}/build")

#find_library(FLYCAPTURE2 flycapture)

find_library(APRILTAG3_LIBRARY
	NAMES apriltag
	PATHS ${APRILTAG3_DIR}/lib
)

message("-->APRILTAG3_LIBRARY: ${APRILTAG3_LIBRARY}")

# Needed libraries
set (EXTRA_LIBS ${APRILTAG3_LIBRARY} ${OpenCV_LIBS} ${VISP_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Define executable apriltag3_exe using messages provided by this package.
add_executable(apriltag3_pub
    ${APRILTAG3_SRC_FOLDER}/apriltag3_main.cpp
    ${APRILTAG3_SRC_FOLDER}/apriltag3_ros.cpp
    # ${APRILTAG3_SRC_FOLDER}/utils.cpp
)

add_dependencies(apriltag3_pub apriltag_msgs_msgs_gencpp ${catkin_EXPORTED_TARGETS} ${${apriltag3_main}_EXPORTED_TARGETS} )

# specify which libraries apriltag3_exe links against
target_link_libraries(apriltag3_pub
    ${EXTRA_LIBS}
)

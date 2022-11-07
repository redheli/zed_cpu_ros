cmake_minimum_required(VERSION 2.8.3)
project(zed_cpu_ros)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            roscpp
            tf
            cv_bridge
            image_transport
            camera_info_manager
        )
# set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

# find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )

add_definitions(${EIGEN_DEFINITIONS})

link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
    # INCLUDE_DIRS include
    LIBRARIES 
    CATKIN_DEPENDS
)

# add_library(zed_cpu_ros src/zed_cpu_ros.cpp)
# target_link_libraries(zed_cpu_ros ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Boost_LIBRARIES})
# add_dependencies(zed_cpu_ros arti_msgs_gencpp)

# set zed-open-capture path
set(ZED_OPEN_CAPTURE_INTALL_PATH "${PROJECT_SOURCE_DIR}/zed-open-capture")
set(ZED_OPEN_CAPTURE_LIB "zed_open_capture")
link_directories("${ZED_OPEN_CAPTURE_INTALL_PATH}/lib")
include_directories("${ZED_OPEN_CAPTURE_INTALL_PATH}/include/zed-open-capture")

add_executable(zed_cpu_ros src/zed_cpu_ros.cpp)
target_link_libraries(zed_cpu_ros ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${ZED_OPEN_CAPTURE_LIB})

# install(TARGETS 
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   PATTERN ".svn" EXCLUDE
# )
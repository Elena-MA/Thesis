cmake_minimum_required(VERSION 2.8.3)
project(amcl_doris)
set(ROS_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Debug)
## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        COMPONENTS
            message_filters
            rosbag
            roscpp
            std_srvs
            std_msgs
            tf
            dynamic_reconfigure
            nav_msgs
            std_srvs
            image_transport
            std_msgs
  	    tf
            geometry_msgs
            image_geometry
            tf2
            message_generation
        )

find_package(Boost REQUIRED)
find_package (OpenCV REQUIRED)
find_package (detector REQUIRED)
add_message_files(
	FILES
	pose_error.msg
)
generate_messages(
	DEPENDENCIES
	std_msgs
)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/AMCL.cfg
)

catkin_package(
    CATKIN_DEPENDS
        rosbag
        roscpp
        dynamic_reconfigure
        tf
	tf2
  CATKIN_DEPENDS nav_msgs std_srvs message_runtime
  INCLUDE_DIRS include
  LIBRARIES amcl_sensors amcl_map amcl_pf
  DEPENDS OpenCV detector
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${detector_INCLUDE_DIRS})

add_library(amcl_pf
                    src/amcl_doris/pf/pf.c
                    src/amcl_doris/pf/pf_kdtree.c
                    src/amcl_doris/pf/pf_pdf.c
                    src/amcl_doris/pf/pf_vector.c
                    src/amcl_doris/pf/eig3.c
                    src/amcl_doris/pf/pf_draw.c)

add_library(amcl_map
                    src/amcl_doris/map/map.c
                    src/amcl_doris/map/map_cspace.cpp
                    src/amcl_doris/map/map_range.c
                    src/amcl_doris/map/map_store.c
                    src/amcl_doris/map/map_draw.c)

add_library(amcl_sensors
                    src/amcl_doris/sensors/amcl_sensor.cpp
                    src/amcl_doris/sensors/amcl_odom.cpp
                    src/amcl_doris/sensors/amcl_laser.cpp
		    src/amcl_doris/sensors/amcl_marker.cpp)
target_link_libraries(amcl_sensors amcl_map amcl_pf ${OPENCV_LIBS} ${catkin_LIBRARIES} detector)


add_executable(amcl_doris
                       src/amcl_node.cpp)
add_dependencies(amcl_doris ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} detector)

target_link_libraries(amcl_doris
    amcl_sensors amcl_map amcl_pf
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OPENCV_LIBS} 
    detector
)

install( TARGETS
    amcl_doris amcl_sensors amcl_map amcl_pf
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/amcl_doris/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY examples/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples
)

## Configure Tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # Bags
  catkin_download_test_data(${PROJECT_NAME}_basic_localization_stage_indexed.bag
    http://download.ros.org/data/amcl/basic_localization_stage_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 41fe43af189ec71e5e48eb9ed661a655)
  catkin_download_test_data(${PROJECT_NAME}_global_localization_stage_indexed.bag
    http://download.ros.org/data/amcl/global_localization_stage_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 752f711cf4f6e8d1d660675e2da096b0)
  catkin_download_test_data(${PROJECT_NAME}_small_loop_prf_indexed.bag
    http://download.ros.org/data/amcl/small_loop_prf_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 e4ef0fc006872b43f12ed8a7ce7dcd81)
  catkin_download_test_data(${PROJECT_NAME}_small_loop_crazy_driving_prg_indexed.bag
    http://download.ros.org/data/amcl/small_loop_crazy_driving_prg_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 4a58d1a7962914009d99000d06e5939c)
  catkin_download_test_data(${PROJECT_NAME}_texas_greenroom_loop_indexed.bag
    http://download.ros.org/data/amcl/texas_greenroom_loop_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 6e3432115cccdca1247f6c807038e13d)
  catkin_download_test_data(${PROJECT_NAME}_texas_willow_hallway_loop_indexed.bag
    http://download.ros.org/data/amcl/texas_willow_hallway_loop_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 27deb742fdcd3af44cf446f39f2688a8)
  catkin_download_test_data(${PROJECT_NAME}_rosie_localization_stage.bag
    http://download.ros.org/data/amcl/rosie_localization_stage.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 3347bf3835724cfa45e958c5c1846066)

  # Maps
  catkin_download_test_data(${PROJECT_NAME}_willow-full.pgm
    http://download.ros.org/data/amcl/willow-full.pgm
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 b84465cdbbfe3e2fb9eb4579e0bcaf0e)
  catkin_download_test_data(${PROJECT_NAME}_willow-full-0.05.pgm
    http://download.ros.org/data/amcl/willow-full-0.05.pgm
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 b61694296e08965096c5e78611fd9765)

  # Tests
  add_rostest(test/set_initial_pose.xml)
  add_rostest(test/set_initial_pose_delayed.xml)
  add_rostest(test/basic_localization_stage.xml)
  add_rostest(test/small_loop_prf.xml)
  add_rostest(test/small_loop_crazy_driving_prg.xml)
  add_rostest(test/texas_greenroom_loop.xml)
  add_rostest(test/rosie_multilaser.xml)
  add_rostest(test/texas_willow_hallway_loop.xml)

# Not sure when or if this actually passed.
#
# The point of this is that you start with an even probability
# distribution over the whole map and the robot localizes itself after
# some number of iterations of sensing and motion.
#
#  add_rostest(test/global_localization_stage.xml)
endif()

cmake_minimum_required(VERSION 2.8.3)
project(localization)

find_package(catkin REQUIRED
        COMPONENTS
            message_filters
            rosbag
            roscpp
            std_srvs
            tf
            nav_msgs
        )

find_package(Boost REQUIRED)


catkin_package(
    CATKIN_DEPENDS
        rosbag
        roscpp
        tf
  CATKIN_DEPENDS nav_msgs std_srvs
  INCLUDE_DIRS include
  LIBRARIES amcl_sensors amcl_map amcl_pf
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(amcl_pf
                    src/particlefilter.c
                    src/pf_kdtree.c
                    src/pf_pdf.c
                    src/pf_vector.c
                    src/eig3.c)

add_library(amcl_map
                    src/map.c
                    src/map_cspace.cpp
                    src/map_range.c
                    src/map_store.c)

add_library(amcl_sensors
                    src/sensor.cpp
                    src/odom.cpp
                    src/laser.cpp)
target_link_libraries(amcl_sensors amcl_map amcl_pf)


add_executable(localization
                       src/localization_node.cpp)
add_dependencies(localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(localization
    amcl_sensors amcl_map amcl_pf
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

install( TARGETS
    localization amcl_sensors amcl_map amcl_pf
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


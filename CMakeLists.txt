cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME droneAltitudeFiltering)
project(${PROJECT_NAME})

add_definitions(-std=c++11)


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries




set(DRONE_ALTITUDE_FILTERING_SOURCE_DIR
	src/sources)
	
set(DRONE_ALTITUDE_FILTERING_INCLUDE_DIR
	src/include

	)

set(DRONE_ALTITUDE_FILTERING_SOURCE_FILES

	#General
	${DRONE_ALTITUDE_FILTERING_SOURCE_DIR}/droneAltitudeFiltering.cpp
	

	
	)
	
set(DRONE_ALTITUDE_FILTERING_HEADER_FILES

	#General
	${DRONE_ALTITUDE_FILTERING_INCLUDE_DIR}/droneAltitudeFiltering.h
	
	)


find_package(catkin REQUIRED
                COMPONENTS roscpp  droneMsgsROS droneModuleROS std_msgs mavros_msgs)


catkin_package(
        CATKIN_DEPENDS roscpp  droneMsgsROS droneModuleROS std_msgs mavros_msgs
  )

find_package( PkgConfig REQUIRED)
pkg_check_modules( EIGEN3 REQUIRED eigen3 )

include_directories(include
        ${EIGEN3_INCLUDE_DIR}
        "/usr/include/eigen3"
)

include_directories(${DRONE_ALTITUDE_FILTERING_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})


add_library(droneAltitudeFiltering_lib ${DRONE_ALTITUDE_FILTERING_SOURCE_FILES} ${DRONE_ALTITUDE_FILTERING_HEADER_FILES})
add_dependencies(droneAltitudeFiltering_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(droneAltitudeFiltering_lib ${catkin_LIBRARIES})



add_executable(droneAltitudeFiltering ${DRONE_ALTITUDE_FILTERING_SOURCE_DIR}/droneAltitudeFilteringNode.cpp)
add_dependencies(droneAltitudeFiltering ${catkin_EXPORTED_TARGETS})
target_link_libraries(droneAltitudeFiltering droneAltitudeFiltering_lib)
target_link_libraries(droneAltitudeFiltering ${catkin_LIBRARIES})



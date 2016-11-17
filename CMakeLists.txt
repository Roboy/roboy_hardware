cmake_minimum_required(VERSION 2.8.3)
project(roboy_hardware)

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../flexrayusbinterface/cmake)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/devel/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/devel/${PROJECT_NAME})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")
SET(CMAKE_BUILD_TYPE Debug)

#==============================================================================
# Dependencies
#==============================================================================
# ros
find_package(catkin REQUIRED COMPONENTS flexrayusbinterface common_utilities
		roscpp controller_manager std_msgs)

# libftd2xx
find_package(libftd2xx)

# pthread
find_package (Threads)

catkin_package(DEPENDS libftd2xx
		CATKIN_DEPENDS common_utilities flexrayusbinterface message_runtime)

message(STATUS "HERE ${catkin_INCLUDE_DIRS} ")

include_directories(
	${catkin_INCLUDE_DIRS} 
	include
)
#==============================================================================
# Application
#==============================================================================
add_executable(roboy src/main.cpp src/roboy.cpp)
add_dependencies(roboy common_utilities_gencpp)
target_link_libraries(roboy ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES}
        ${libftd2xx_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} -ldl)
#==============================================================================
# Installation
#==============================================================================
install( TARGETS
        roboy
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
	package.xml
  	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY 
	launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

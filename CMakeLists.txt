cmake_minimum_required(VERSION 2.8.2)
project(wgs_conversions)
 
 # suppress policy warnings on newer versions of cmake
IF (NOT(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} LESS 3.1))
  cmake_policy(SET CMP0053 OLD)
ENDIF (NOT(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} LESS 3.1))


IF (NOT(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} LESS 3.0))
  cmake_policy(SET CMP0042 OLD)
ENDIF (NOT(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} LESS 3.0))

# enable C++11 support
set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++0x")
set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
# set(CMAKE_CXX_FLAGS "-std=gnu++11")
set(CMAKE_CXX_FLAGS "-std=c++0x")


option(BUILD_WITH_CATKIN "Build with Catkin?" OFF)
if (DEFINED CATKIN_DEVEL_PREFIX)
  SET(BUILD_WITH_CATKIN ON)
endif(DEFINED CATKIN_DEVEL_PREFIX)
option(BUILD_ROS_NODE "Build ROS node." ON)

option(BUILD_EXAMPLE "Build Example." OFF)


if (BUILD_WITH_CATKIN)
  set(catkin_components )
  if (BUILD_EXAMPLE)
    set(BUILD_ROS_NODE ON)
  endif (BUILD_EXAMPLE)

  if (BUILD_ROS_NODE)
    list(APPEND catkin_components roslib roscpp std_msgs message_generation message_runtime)
  endif (BUILD_ROS_NODE)

  ## Find catkin macros and libraries
	## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
	find_package(catkin COMPONENTS ${catkin_components})

	 #  add_message_files(
  # FILES
  # g35can_engine_data.msg
  # g35can_steer_angle.msg
  # g35can_wheel_speed.msg
  # )
  
  add_service_files(FILES WgsConversion.srv)

  generate_messages()

  # generate_messages(
  # DEPENDENCIES
  # std_msgs
  # geometry_msgs
  # )

	catkin_package(
	  INCLUDE_DIRS include
	  LIBRARIES wgs_conversions
	  CATKIN_DEPENDS ${catkin_components}
	)





else()
	SET(CATKIN_PACKAGE_LIB_DESTINATION "${CMAKE_INSTALL_PREFIX}/lib")
	SET(CATKIN_PACKAGE_BIN_DESTINATION "${CMAKE_INSTALL_PREFIX}/bin")
	SET(CATKIN_PACKAGE_INCLUDE_DESTINATION "${CMAKE_INSTALL_PREFIX}/include")
	# find_package(pcan_interface REQUIRED)
	# set(catkin_INCLUDE_DIRS ${fpcan_interface_INCLUDE_DIRS})
	# set(catkin_LIBRARIES ${pcan_interface_LIBRARIES})
endif (BUILD_WITH_CATKIN)

###########
## Build ##
###########



## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})



# Declare a cpp library
add_library(wgs_conversions
            src/wgs_conversions.cpp
            include/wgs_conversions/wgs_conversions.h)

target_link_libraries(wgs_conversions ${catkin_LIBRARIES})


# Build ROS node
if (BUILD_ROS_NODE)
	# Declare a cpp executable
	add_executable(wgs_conversions_server src/wgs_conversions_server.cpp)
  	add_dependencies(wgs_conversions_server wgs_conversions_gencpp ${catkin_EXPORTED_TARGETS})
	target_link_libraries(wgs_conversions_server wgs_conversions ${catkin_LIBRARIES})
endif (BUILD_ROS_NODE)

# Build Example
if (BUILD_EXAMPLE)
  # Declare a cpp executable
  add_executable(wgs_conversions_client example/wgs_conversions_client.cpp)
    add_dependencies(wgs_conversions_client wgs_conversions_gencpp ${catkin_EXPORTED_TARGETS})
    target_link_libraries(wgs_conversions_client ${catkin_LIBRARIES})
endif (BUILD_EXAMPLE)


#############
## Install ##
#############


# Mark executables and/or libraries for installation
if(BUILD_WITH_CATKIN)
  set (target_list wgs_conversions)
  if (BUILD_ROS_NODE)
    list(APPEND target_list wgs_conversions_server)
  endif (BUILD_ROS_NODE)
  if (BUILD_EXAMPLE)
    list(APPEND target_list wgs_conversions_client)
    install(DIRECTORY example DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  endif (BUILD_EXAMPLE)
  install(TARGETS ${target_list}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  # Mark cpp header files for installation
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
  )
else(BUILD_WITH_CATKIN) # do a classical install
  install(TARGETS ${LIB_NAME}
    ARCHIVE DESTINATION /usr/local/lib
    LIBRARY DESTINATION /usr/local/lib
    RUNTIME DESTINATION /usr/local/bin/${PROJECT_NAME}
  )
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION /usr/local/include/${PROJECT_NAME}
  )
  set(CMAKE_MODULE_PATH ${CMAKE_ROOT}/Modules)
  install(FILES FindWgsConversions.cmake DESTINATION ${CMAKE_MODULE_PATH})
endif (BUILD_WITH_CATKIN)


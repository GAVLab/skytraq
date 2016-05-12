## Project Setup
cmake_minimum_required(VERSION 2.8.2)
project(skytraq)

if(${CMAKE_MAJOR_VERSION} GREATER 2)
  cmake_policy(SET CMP0042 OLD)
endif(${CMAKE_MAJOR_VERSION} GREATER 2)

option(BUILD_WITH_CATKIN "Build with Catkin?" OFF)

if (DEFINED CATKIN_DEVEL_PREFIX)
  SET(BUILD_WITH_CATKIN ON)
endif(DEFINED CATKIN_DEVEL_PREFIX)


if(BUILD_WITH_CATKIN)
  message(STATUS "Building with Catkin")

  set(catkin_components serial)

  ## Find catkin macros and libraries
  find_package(catkin COMPONENTS ${catkin_components})

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES skytraq
    CATKIN_DEPENDS ${catkin_components}
    DEPENDS Boost
  )

else(BUILD_WITH_CATKIN)
  message(STATUS "Building without Catkin")
  
  SET(CATKIN_PACKAGE_LIB_DESTINATION "${CMAKE_INSTALL_PREFIX}/lib")
  SET(CATKIN_PACKAGE_BIN_DESTINATION "${CMAKE_INSTALL_PREFIX}/bin")
  SET(CATKIN_GLOBAL_INCLUDE_DESTINATION "${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}")
  SET(CATKIN_PACKAGE_INCLUDE_DESTINATION "${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}")

  find_package(serial REQUIRED)
  set(catkin_LIBRARIES ${serial_LIBRARIES})
  set(catkin_INCLUDE_DIRS ${serial_INCLUDE_DIRS})

endif(BUILD_WITH_CATKIN)

find_package(Boost REQUIRED COMPONENTS system thread)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Declare a cpp library
add_library(skytraq
            src/skytraq.cpp
            include/skytraq/skytraq.h
            include/skytraq/skytraq_structures.h
)

target_link_libraries (skytraq ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## Build Examples
option(BUILD_EXAMPLES OFF "Build skytraq examples.")
IF(BUILD_EXAMPLES)


ENDIF(BUILD_EXAMPLES)

## Build tests
IF(SKYTRAQ_BUILD_TESTS)
    
ENDIF(SKYTRAQ_BUILD_TESTS)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
# Mark executables and/or libraries for installation
install(TARGETS skytraq
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

set(CMAKE_MODULE_PATH ${CMAKE_ROOT}/Modules)
install(FILES Findskytraq.cmake DESTINATION ${CMAKE_MODULE_PATH})

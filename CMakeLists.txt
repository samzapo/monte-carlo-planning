cmake_minimum_required(VERSION 2.8.3)

project(Virtualization)

include (CheckIncludeFiles)
include (CheckLibraryExists)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/CMakeModules)

# setup include directories
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#set(CMAKE_CXX_FLAGS "-lboost_program_options")

#find BOOST libraries
set(Boost_ADDITIONAL_VERSIONS "1.55.0" "1.56.0")
find_package(Boost COMPONENTS program_options REQUIRED)
link_directories(${BOOST_LIBRARY_DIRS})
include_directories(${BOOST_INCLUDE_DIR})

#add ravelin
#find_package(Pacer REQUIRED)
#link_directories(${PACER_LIBRARY_DIR})
#include_directories(${PACER_INCLUDE_DIR})

#add ravelin
find_package(Ravelin REQUIRED)
link_directories(${RAVELIN_LIBRARY_DIR})
include_directories(${RAVELIN_INCLUDE_DIR})

#add moby
find_package(Moby REQUIRED)
link_directories(${MOBY_LIBRARY_DIR})
include_directories(${MOBY_INCLUDE_DIR})

#add GLConsole
find_package(LibXml2 REQUIRED)
link_directories(${LIBXML2_LIBRARY_DIR})
include_directories(${LIBXML2_INCLUDE_DIR})

# fix the C++ linking error on 64-bit Linux
set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl")

include_directories(
                    include
                    /usr/include
                    /usr/local/include
                    )

link_directories(
                  /usr/lib
                  /usr/local/lib
                  )

set(EXPERIMENT_SOURCES
            src/main.cpp
            src/Experiment.cpp
           )

set(SAMPLE_SOURCES
            src/Sample.cpp
            src/Moby.cpp
           )
         
#defines        
add_definitions (-DUSE_OSG)
find_package (osg)
find_package (osgViewer)
find_package (osgDB)
find_package (osgGA)
find_package (osgFX)
find_package (osgUtil)
find_package (OpenThreads)
link_directories(${OSG_LIBRARY_DIRS})
include_directories(${OSG_INCLUDE_DIR})

add_executable(${PROJECT_NAME} ${EXPERIMENT_SOURCES})
add_executable(sample.bin ${SAMPLE_SOURCES})
  
IF(OSG_FOUND)
  target_link_libraries (sample.bin ${OSG_LIBRARIES})
  target_link_libraries (sample.bin ${OSGVIEWER_LIBRARIES})
  target_link_libraries (sample.bin ${OSGDB_LIBRARIES})
  target_link_libraries (sample.bin ${OSGGA_LIBRARIES})
  target_link_libraries (sample.bin ${OPENTHREADS_LIBRARY})
  target_link_libraries (sample.bin ${OSGUTIL_LIBRARIES})
  target_link_libraries (sample.bin ${OSGMANIPULATOR_LIBRARIES})
  target_link_libraries (sample.bin OpenThreads)
  add_definitions (-DUSE_OSG_DISPLAY)
ENDIF(OSG_FOUND)

         #add_library(${PROJECT_NAME} ${SOURCES})

# LibXml2
IF(LIBXML2_FOUND)
  target_link_libraries (sample.bin ${LIBXML2_LIBRARIES})
ENDIF(LIBXML2_FOUND)

target_link_libraries(${PROJECT_NAME} ${MOBY_LIBRARIES} ${RAVELIN_LIBRARIES} ${BOOST_LIBRARIES} gsl boost_program_options)
target_link_libraries(sample.bin ${MOBY_LIBRARIES} ${RAVELIN_LIBRARIES} ${BOOST_LIBRARIES} gsl boost_program_options)

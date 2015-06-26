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
find_package(Boost COMPONENTS program_options thread REQUIRED)
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

set(SOURCES
            src/main.cpp
            src/Experiment.cpp
            src/Moby.cpp
           )

         #add_library(${PROJECT_NAME} ${SOURCES})
add_executable(${PROJECT_NAME} ${SOURCES})

# LibXml2
IF(LIBXML2_FOUND)
  target_link_libraries (${PROJECT_NAME} ${LIBXML2_LIBRARIES})
ENDIF(LIBXML2_FOUND)

target_link_libraries(${PROJECT_NAME} ${MOBY_LIBRARIES} ${RAVELIN_LIBRARIES} ${BOOST_LIBRARIES} gsl boost_program_options)
if(APPLE OR WIN32)
  target_link_libraries(${PROJECT_NAME} boost_thread-mt boost_system-mt)
else()
  target_link_libraries(${PROJECT_NAME} boost_thread boost_system)
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

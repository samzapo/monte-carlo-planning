# Find PACER header and library.
#

# This module defines the following uncached variables:
#  PACER_FOUND, if false, do not try to use PACER.
#  PACER_INCLUDE_DIRS, where to find PACER/PACER_a.h.
#  PACER_LIBRARIES, the libraries to link against to use the PACER library
#  PACER_LIBRARY_DIRS, the directory where the PACER library is found.

find_path(
  PACER_INCLUDE_DIR
  NAMES Pacer/controller.h
  PATHS /usr/local/include /usr/include
)

if( PACER_INCLUDE_DIR )
  find_library(
    PACER_LIBRARY
    NAMES libPacer.so libPacer.dylib libPacer.a 
    PATHS /usr/local/lib /usr/lib
  )
  if( PACER_LIBRARY )
    set(PACER_LIBRARY_DIR "")
    get_filename_component(PACER_LIBRARY_DIRS ${PACER_LIBRARY} PATH)
    # Set uncached variables as per standard.
    set(PACER_FOUND ON)
    set(PACER_INCLUDE_DIRS ${PACER_INCLUDE_DIR})
    set(PACER_LIBRARIES ${PACER_LIBRARY})
  endif(PACER_LIBRARY)
else(PACER_INCLUDE_DIR)
  message(STATUS "FindPacer: Could not find controller.h")
endif(PACER_INCLUDE_DIR)
	    
if(PACER_FOUND)
  if(NOT PACER_FIND_QUIETLY)
    message(STATUS "FindPacer: Found both controller.h and libPacer")
  endif(NOT PACER_FIND_QUIETLY)
else(PACER_FOUND)
  if(PACER_FIND_REQUIRED)
    message(FATAL_ERROR "FindPacer: Could not find controller.h and/or libPacer")
  endif(PACER_FIND_REQUIRED)
endif(PACER_FOUND)

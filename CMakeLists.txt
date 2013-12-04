# Copyright (C) 2008 Wang Rui <wangray84@gmail.com>
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.

# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.

# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#
# Authors:
#   Wang Rui <wangray84@gmail.com>

PROJECT(osgModeling)
SET(PROJECT_NAME osgModeling)

SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules/")
CMAKE_MINIMUM_REQUIRED(VERSION 2.4.7)

IF(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)

SET(OSGMODELING_MAJOR_VERSION 0)
SET(OSGMODELING_MINOR_VERSION 1)
SET(OSGMODELING_PATCH_VERSION 1)
SET(OSGMODELING_VERSION ${OSGMODELING_MAJOR_VERSION}.${OSGMODELING_MINOR_VERSION}.${OSGMODELING_PATCH_VERSION})

FIND_PACKAGE(OpenSceneGraph)
FIND_PACKAGE(Doxygen)

SET(OSG_DEBUG_POSTFIX "d" CACHE STRING "use postfix to link with osg with debug")

IF(WIN32)
    IF(MSVC)
        OPTION(WIN32_USE_MP "Set to ON to build OpenSceneGraph with the /MP option (Visual Studio 2005 and above)." OFF)
        MARK_AS_ADVANCED(WIN32_USE_MP)
        IF(WIN32_USE_MP)
            SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
        ENDIF(WIN32_USE_MP)

        # More MSVC specific compilation flags
        ADD_DEFINITIONS(-D_SCL_SECURE_NO_WARNINGS)
        ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)
    ENDIF(MSVC)
ENDIF(WIN32)

SET(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} CACHE STRING "You may add additional search paths here. Use ; to separate multiple paths.")
SET(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} CACHE STRING "You may add additional search paths here. Use ; to separate multiple paths.")

IF (NOT WIN32)
    SET(CMAKE_CXX_FLAGS "-W -Wall -Wno-unused")
ENDIF(NOT WIN32)

INCLUDE_DIRECTORIES(include ${CMAKE_INCLUDE_PATH} ${OPENSCENEGRAPH_INCLUDE_DIR})
LINK_DIRECTORIES(${CMAKE_LIBRARY_PATH} ${OPENSCENEGRAPH_LIB_DIR})

ADD_SUBDIRECTORY(src/osgModeling)

OPTION(BUILD_PLUGINS "Enable to build osgModeling support for .osg/.ive format" OFF)
IF(BUILD_PLUGINS)
    ADD_SUBDIRECTORY(src/osgPlugins/osgModeling)
ENDIF(BUILD_PLUGINS)

OPTION(BUILD_EXAMPLES "Enable to build osgModeling Examples" ON)
IF(BUILD_EXAMPLES)
    ADD_SUBDIRECTORY(examples/osgmodelingbasic)
    ADD_SUBDIRECTORY(examples/osgmodelingnurbs)
    ADD_SUBDIRECTORY(examples/osgmodelingbsptree)
    ADD_SUBDIRECTORY(examples/osgmodelingboolean)
    ADD_SUBDIRECTORY(examples/osgmodelingsubd)
    ADD_SUBDIRECTORY(data)
ENDIF(BUILD_EXAMPLES)

OPTION(BUILD_DOCUMENTATION "Build osgModeling documentation" OFF)
IF(BUILD_DOCUMENTATION)
    OPTION(BUILD_HTMLHELP "Also build HTML help file (.chm)" OFF)
    IF(BUILD_HTMLHELP)
        SET(GENERATE_HTMLHELP YES)
        
        FIND_PROGRAM(HHC_FILE hhc.exe
          PATHS
          $ENV{PATH}
        )
    ELSE(BUILD_HTMLHELP)
        SET(GENERATE_HTMLHELP NO)
    ENDIF(BUILD_HTMLHELP)

    IF(DOT)
        SET(HAVE_DOT YES)
    ELSE(DOT)
        SET(HAVE_DOT NO)
    ENDIF(DOT)
    
    CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/doc/Doxyfiles/doxyfile.cmake
        ${PROJECT_BINARY_DIR}/doc/doxyfile
    )
    ADD_CUSTOM_TARGET(DoxygenDoc ${DOXYGEN}
        ${PROJECT_BINARY_DIR}/doc/doxyfile
    )
ENDIF(BUILD_DOCUMENTATION)

CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)
ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

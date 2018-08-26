cmake_minimum_required(VERSION 2.8)


## Project branding, version and package mantainer
project(grsim_scramble_plugin)
set(VERSION "0.0.1")
#set(VENDOR "Parsian")
set(MAINTAINER "Shohei Fujii <fujii.shohei@gmail.com>")


# some utils and helper vars
string(TOLOWER ${CMAKE_PROJECT_NAME} CMAKE_PROJECT_NAME_LOWER)
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)
#include(${PROJECT_SOURCE_DIR}/cmake/Utils.cmake)
#standard_config()
#standard_paths(${PROJECT_SOURCE_DIR} bin lib)

if("$ENV{GRSIM_PLUGIN_PATH}" STREQUAL "")
  set(ENV{GRSIM_PLUGIN_PATH} "$ENV{HOME}/.grsim/plugins/")
endif()


# definitions for knowing the OS from the code
if(MSVC)
    add_definitions(-DHAVE_MSVC)
endif()
if(WIN32)
    add_definitions(-DHAVE_WINDOWS)
endif()
if(UNIX)
    add_definitions(-DHAVE_UNIX)
    if(APPLE)
        add_definitions(-DHAVE_MACOSX)
    else()
        #TODO: fix this, say we have FreeBSD, that's not linux
        add_definitions(-DHAVE_LINUX)
    endif()
endif()

# set explicitly the c++ standard to use
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++03")

# add src dir to included directories
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${CMAKE_INSTALL_PREFIX}/include)


## Handling depenendcies

# we will append all libs to this var
set(libs)

# Boost
find_package(Boost REQUIRED) #dll
include_directories(${Boost_INCLUDE_DIR})
#list(APPEND libs ${Boost_LIBRARIES} dl)

# OpenGL
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})
list(APPEND libs ${OPENGL_LIBRARIES})
#find_package(GLUT REQUIRED)
#include_directories(${GLUT_INCLUDE_DIR})

# Qt
set(QT_USE_QTNETWORK true)
set(QT_USE_QTOPENGL true)
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
list(APPEND libs ${QT_LIBRARIES})
# find_package(Qt5 REQUIRED COMPONENTS Gui Network OpenGL)
# if(${QT5_NOTFOUND})
#   message(STATUS "qt5 not found!")
# else()
#   message(STATUS "qt5 found!")
# endif()
# list(APPEND libs Qt5::Gui Qt5::Network Qt5::OpenGL)

# ODE
# find_package(ODE REQUIRED)
# include_directories(${ODE_INCLUDE_DIRS})
# list(APPEND libs ${ODE_LIBRARIES})
# #TODO: make this an option
# option(DOUBLE_PRECISION "Use double precision? If not single precision will be used." ON)
#   if(DOUBLE_PRECISION)
#     if(APPLE)
#       add_definitions(-DdIDEDOUBLE)
#     else()
#       add_definitions(-DdDOUBLE)
#     endif()
#   else()
#     if(APPLE)
#       add_definitions(-DdIDESINGLE)
#     else()
#       add_definitions(-DdSINGLE)
#     endif()
#   endif()

set(SOURCES
    src/attacker_robot.cpp
)

set(HEADERS
)

# files to be compiled
set(srcs
  #${CONFIG_FILES}
    ${PROTO_CPP}
    ${PROTO_H}
    ${HEADERS}
    ${SOURCES}
)

message(STATUS "sources: ${SOURCES}")
file(GLOB CONFIG_FILES "config/*.ini")
set_source_files_properties(${CONFIG_FILES}  PROPERTIES MACOSX_PACKAGE_LOCATION "config")

if(APPLE)
    set(plugin ${CMAKE_PROJECT_NAME})
    add_library(${plugin} SHARED MACOSX_BUNDLE ${srcs})
    target_link_libraries(${plugin} ${libs})
    install(TARGETS ${plugin} DESTINATION "$ENV{GRSIM_PLUGIN_PATH}")
    #set(MACOSX_BUNDLE_ICON_FILE "${PROJECT_SOURCE_DIR}/resources/icons/grsim.icns")
    #set(MACOSX_BUNDLE_SHORT_VERSION_STRING ${VERSION})
    #set(MACOSX_BUNDLE_VERSION ${VERSION})
    #set(MACOSX_BUNDLE_LONG_VERSION_STRING "Version ${VERSION}")
    #set(BUNDLE_APP ${PROJECT_SOURCE_DIR}/bin/${app}.app)
    #install(
    #    CODE "
    #    include(BundleUtilities)
    #    fixup_bundle(\"${BUNDLE_APP}\"   \"\"   \"/opt/local/lib;/usr/local/lib\")"
    #    COMPONENT Runtime)
    #set(CPACK_GENERATOR "DragNDrop" "TGZ")
elseif(WIN32)
    set(plugin ${CMAKE_PROJECT_NAME})
    add_library(${plugin} SHARED WIN32 ${srcs})
    target_link_libraries(${plugin} ${libs})
    install(TARGETS ${plugin} DESTINATION "$ENV{GRSIM_PLUGIN_PATH}")
    install(DIRECTORY config DESTINATION .)
    #install(DIRECTORY lib DESTINATION .
    #        FILES_MATCHING PATTERN "*.dll")
    #set(CPACK_PACKAGE_EXECUTABLES ${app} ${app})
else()
    set(plugin ${CMAKE_PROJECT_NAME_LOWER})
    add_library(${plugin} SHARED ${srcs})
    target_link_libraries(${plugin} ${libs})
    install(TARGETS ${plugin} DESTINATION "$ENV{GRSIM_PLUGIN_PATH}")
    install(DIRECTORY config DESTINATION share/grsim)
endif()


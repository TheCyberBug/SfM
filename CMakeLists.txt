cmake_minimum_required(VERSION 3.28)
project(SfM)

set(CMAKE_CXX_STANDARD 17)

set(RESOURCE_FOLDER "${CMAKE_SOURCE_DIR}/data")
file(COPY ${RESOURCE_FOLDER} DESTINATION ${CMAKE_BINARY_DIR})

set(OPENCV_BUILD_3RDPARTY_LIBS ON CACHE BOOL "Build 3rd party libraries")
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package( VTK REQUIRED )
include(${VTK_USE_FILE})

add_executable(SfM main.cpp
        helper/DataOps.cpp
        helper/DataOps.h
        helper/Features.cpp
        helper/Features.h
        helper/config.h
        helper/extractors.cpp
        helper/extractors.h
        helper/ccv.cpp
        helper/ccv.h
        helper/object.cpp
        helper/object.h
        helper/visualiser.cpp
        helper/visualiser.h
        helper/cameraCalibration.cpp
        helper/cameraCalibration.h
        SfMPipeline.cpp
        SfMPipeline.h
        helper/ccv/EssentialMatrix.cpp
        helper/ccv/EssentialMatrix.h
        helper/ccv/RecoverPose.cpp
        helper/ccv/RecoverPose.h
        helper/templates/templates.h
        UI/menu.cpp
        UI/menu.h
        UI/uiconfig.h
)

target_link_libraries( SfM ${OpenCV_LIBS} ${VTK_LIBRARIES})

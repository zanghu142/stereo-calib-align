cmake_minimum_required(VERSION 3.16)

project(stereo_csi_calibrator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(MSVC)
    add_compile_options(/utf-8 /EHsc)
endif()

find_package(OpenCV 4.5 REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig QUIET)
find_package(Freetype QUIET)
find_package(OpenGL QUIET COMPONENTS EGL GLES2)

add_executable(stereo_csi_calibrator
    src/main.cpp
    src/calibrator.cpp
    src/camera_pose.cpp
    src/csi_camera.cpp
    src/hardware_preview.cpp
)

target_link_libraries(stereo_csi_calibrator PRIVATE ${OpenCV_LIBS} Threads::Threads)

if(Freetype_FOUND)
    target_compile_definitions(stereo_csi_calibrator PRIVATE HAVE_FREETYPE)
    target_link_libraries(stereo_csi_calibrator PRIVATE Freetype::Freetype)
endif()

if(OpenGL_EGL_FOUND AND OpenGL_GLES2_FOUND)
    target_compile_definitions(stereo_csi_calibrator PRIVATE HAVE_EGL_OPENGL_VIEW)
    target_link_libraries(stereo_csi_calibrator PRIVATE OpenGL::EGL OpenGL::GLES2)
elseif(PkgConfig_FOUND)
    pkg_check_modules(EGL_GLES QUIET egl glesv2)
    if(EGL_GLES_FOUND)
        target_compile_definitions(stereo_csi_calibrator PRIVATE HAVE_EGL_OPENGL_VIEW)
        target_include_directories(stereo_csi_calibrator PRIVATE ${EGL_GLES_INCLUDE_DIRS})
        target_link_directories(stereo_csi_calibrator PRIVATE ${EGL_GLES_LIBRARY_DIRS})
        target_link_libraries(stereo_csi_calibrator PRIVATE ${EGL_GLES_LIBRARIES})
        target_compile_options(stereo_csi_calibrator PRIVATE ${EGL_GLES_CFLAGS_OTHER})
    endif()
endif()

if(UNIX)
    find_package(X11 QUIET)
    if(X11_FOUND)
        target_compile_definitions(stereo_csi_calibrator PRIVATE HAVE_X11)
        target_link_libraries(stereo_csi_calibrator PRIVATE ${X11_LIBRARIES})
        target_include_directories(stereo_csi_calibrator PRIVATE ${X11_INCLUDE_DIR})
    endif()
endif()

if(PkgConfig_FOUND)
    pkg_check_modules(GSTREAMER QUIET gstreamer-1.0 gstreamer-app-1.0)
    if(GSTREAMER_FOUND)
        target_sources(stereo_csi_calibrator PRIVATE src/gst_stereo.cpp)
        target_compile_definitions(stereo_csi_calibrator PRIVATE HAVE_GSTREAMER_PREVIEW)
        target_include_directories(stereo_csi_calibrator PRIVATE ${GSTREAMER_INCLUDE_DIRS})
        target_link_directories(stereo_csi_calibrator PRIVATE ${GSTREAMER_LIBRARY_DIRS})
        target_link_libraries(stereo_csi_calibrator PRIVATE ${GSTREAMER_LIBRARIES})
        target_compile_options(stereo_csi_calibrator PRIVATE ${GSTREAMER_CFLAGS_OTHER})
    endif()
endif()

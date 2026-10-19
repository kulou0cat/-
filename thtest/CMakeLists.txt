cmake_minimum_required(VERSION 3.20)
project(GAMES201_TimeIntegrator LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 已根据这台电脑的实际安装位置配置。换电脑时只需修改这两个路径。
set(EIGEN_ROOT "D:/games101/eigen-3.4.0" CACHE PATH "Directory containing Eigen/Dense")
set(OpenCV_DIR "D:/games101/opencv/build" CACHE PATH "Directory containing OpenCVConfig.cmake")
if(NOT EXISTS "${EIGEN_ROOT}/Eigen/Dense")
    message(FATAL_ERROR "Please set EIGEN_ROOT to the directory containing Eigen/Dense")
endif()
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc imgcodecs)

add_executable(time_integrator main.cpp integrator.h)
target_include_directories(time_integrator SYSTEM PRIVATE "${EIGEN_ROOT}" ${OpenCV_INCLUDE_DIRS})
target_link_libraries(time_integrator PRIVATE ${OpenCV_LIBS})
if(MSVC)
    target_compile_options(time_integrator PRIVATE /utf-8 /W4 /external:W0)
endif()
# 把匹配 Debug/Release 的 OpenCV DLL 放到 exe 旁边，无须改系统 PATH。
foreach(lib IN LISTS OpenCV_LIBS)
    if(TARGET ${lib})
        get_target_property(lib_type ${lib} TYPE)
        if(lib_type STREQUAL "SHARED_LIBRARY")
            add_custom_command(TARGET time_integrator POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "$<TARGET_FILE:${lib}>" "$<TARGET_FILE_DIR:time_integrator>"
                VERBATIM)
        endif()
    endif()
endforeach()

enable_testing()
add_executable(integrator_checks tests.cpp integrator.h)
target_include_directories(integrator_checks SYSTEM PRIVATE "${EIGEN_ROOT}")
if(MSVC)
    target_compile_options(integrator_checks PRIVATE /utf-8 /W4 /external:W0)
endif()
add_test(NAME integrator_checks COMMAND integrator_checks)
set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT time_integrator)

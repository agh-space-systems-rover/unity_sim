cmake_minimum_required(VERSION 3.20)
project(RealSenseNativePlugin VERSION 0.1.0 LANGUAGES CXX)

# Add libjpeg-turbo
add_library(turbojpeg-static STATIC IMPORTED)
set_target_properties(turbojpeg-static PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/libjpeg-turbo/lib64/${CMAKE_STATIC_LIBRARY_PREFIX}turbojpeg${CMAKE_STATIC_LIBRARY_SUFFIX}"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/libjpeg-turbo/include"
)

# Add libhv
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_subdirectory(libhv)

add_library(${PROJECT_NAME} SHARED "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp")
target_link_libraries(${PROJECT_NAME}
    hv_static
    turbojpeg-static
    -static-libgcc
    -static-libstdc++
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 20
)

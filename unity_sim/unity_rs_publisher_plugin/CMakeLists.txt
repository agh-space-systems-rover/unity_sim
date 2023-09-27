cmake_minimum_required(VERSION 3.20)
project(UnityRSPublisherPlugin VERSION 0.1.0 LANGUAGES CXX)

add_library(${PROJECT_NAME} SHARED "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp")
target_link_libraries(${PROJECT_NAME}
    -static-libgcc
    -static-libstdc++
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 20
)

cmake_minimum_required(VERSION 3.20)
project(UnityRSPublisherPlugin LANGUAGES CXX)

add_library(${PROJECT_NAME} SHARED "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp")

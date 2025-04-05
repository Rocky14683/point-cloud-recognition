cmake_minimum_required(VERSION 3.24)

set(TARGET_NAME lidar_vis)
project(${TARGET_NAME} LANGUAGES CXX)

set(CMAKE_C_COMPILER "/opt/homebrew/opt/llvm/bin/clang")
set(CMAKE_CXX_COMPILER "/opt/homebrew/opt/llvm/bin/clang++")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c23 -Wnon-c-typedef-for-linkage")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++23 -Wno-c23-extensions -O3 -Wnon-c-typedef-for-linkage")

set(CMAKE_C_STANDARD 23)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL 1.15 REQUIRED)
find_package(JsonCpp REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(PkgConfig REQUIRED)
pkg_check_modules(MLPACK REQUIRED mlpack)
include_directories(${MLPACK_INCLUDE_DIRS})
link_directories(${MLPACK_LIBRARY_DIRS})

include(FetchContent)
FetchContent_Declare(
        rerun_sdk
        URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip
)
FetchContent_MakeAvailable(rerun_sdk)

include_directories(./include)
include_directories(./include/3D_Ground_Segmentation)
add_executable(${TARGET_NAME} src/main.cpp
        src/3D_Ground_Segmentation/ground_plane_fitting.cpp)
target_link_libraries(${TARGET_NAME} PUBLIC
        rerun_sdk
        ${OpenCV_LIBS}
        Eigen3::Eigen
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${MLPACK_LIBRARIES}
        JsonCpp::JsonCpp
)

add_subdirectory(cnpy)
include_directories(cnpy)
# data collector
add_executable(data_collector
        cnpy
        src/training_data_collector.cpp
        src/3D_Ground_Segmentation/ground_plane_fitting.cpp)
target_link_libraries(data_collector PUBLIC
        cnpy
        rerun_sdk
        ${OpenCV_LIBS}
        Eigen3::Eigen
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${MLPACK_LIBRARIES}
        JsonCpp::JsonCpp
)


# google test

FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)


enable_testing()

file(GLOB TEST_SOURCES test/*.cpp)
add_executable(GTEST ${TEST_SOURCES})

target_include_directories(GTEST PRIVATE include)

target_link_libraries(GTEST PUBLIC gtest gtest_main
        rerun_sdk
        ${OpenCV_LIBS}
        Eigen3::Eigen
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${MLPACK_LIBRARIES}
        JsonCpp::JsonCpp
)

include(GoogleTest)
gtest_discover_tests(GTEST)

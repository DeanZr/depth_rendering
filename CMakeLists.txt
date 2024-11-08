cmake_minimum_required(VERSION 3.10)
project(depth_rendering)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/third_party/library/include)
include_directories(${CMAKE_SOURCE_DIR}/third_party/library/src/include)
include_directories(/usr/local/Cellar/opencv/4.6.0_1)

# Link directories
link_directories(${CMAKE_SOURCE_DIR})

# Source files
file(GLOB_RECURSE PROJECT_SOURCES src/*.cpp)
file(GLOB_RECURSE LIBRARY_SOURCES third_party/library/src/*.cpp)

# Ensure main.cpp is included
set(SOURCES main.cpp ${PROJECT_SOURCES} ${LIBRARY_SOURCES})

# Add OpenCV libraries
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Create the executable
add_executable(depth_rendering main.cpp ${PROJECT_SOURCES} ${LIBRARY_SOURCES})

# Link OpenCV libraries
target_link_libraries(depth_rendering ${OpenCV_LIBS})

# Ensure correct SDK is chosen
execute_process(
    COMMAND xcrun --sdk macosx --show-sdk-path
    OUTPUT_VARIABLE CMAKE_OSX_SYSROOT
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
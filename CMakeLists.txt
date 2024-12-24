cmake_minimum_required(VERSION 3.22)
project(dyn-types)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(include)

set(SOURCES src/Fluid.cpp)

set(TYPES
    ""
    CACHE STRING "Types to precompile")
add_compile_definitions(-DTYPES=${TYPES})

set(SIZES
    ""
    CACHE STRING "Sizes to precompile")
add_compile_definitions(-DSIZES=${SIZES})

add_executable(${PROJECT_NAME} ${SOURCES})
add_executable(${PROJECT_NAME}_og src/fluid_og.cpp)

# Dependencies
include(FetchContent)

# cxxopts
message(STATUS "Fetching dependency: cxxopts")
FetchContent_Declare(
  cxxopts
  GIT_REPOSITORY https://github.com/jarro2783/cxxopts
  GIT_TAG 3bf268481da8208d171d8908e6491459de3651d7) # 3.2.0
FetchContent_MakeAvailable(cxxopts)
target_link_libraries(${PROJECT_NAME} PRIVATE cxxopts)

# boost
if(0)
  message(STATUS "Fetching dependency: boost (may take a while)")
  add_compile_definitions(BOOST_ENABLE_ASSERT_DEBUG_HANDLER)

  set(BOOST_ENABLE_CMAKE ON)
  set(BOOST_INCLUDE_LIBRARIES assert;stacktrace;range)
  FetchContent_Declare(
    Boost
    URL https://github.com/boostorg/boost/releases/download/boost-1.86.0/boost-1.86.0-cmake.tar.xz
        DOWNLOAD_EXTRACT_TIMESTAMP
        ON)

  FetchContent_MakeAvailable(Boost)
endif()

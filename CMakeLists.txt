cmake_minimum_required(VERSION 3.0)

if (DEFINED ENV{VCPKG_ROOT})
  set(VCPKG_CMAKE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
  list(APPEND CMAKE_TOOLCHAIN_FILE ${VCPKG_CMAKE})
endif ()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(RLCPP "${CMAKE_CURRENT_SOURCE_DIR}/../../raylib-cpp-3.5.0-beta2")
set(RLEXTRAS "${CMAKE_CURRENT_SOURCE_DIR}/../../raylib-extras")
set(n gameai-assignment1)

project(${n})

find_package(raylib CONFIG REQUIRED)

add_executable(${n} ${n}.cpp)
target_include_directories(${n} PRIVATE ${RAYLIB_INCLUDE_DIRS})
target_include_directories(${n} PRIVATE ${RLCPP}/include ${RLEXTRAS})
target_link_libraries(${n} PRIVATE ${RAYLIB_LIBRARIES})

if (UNIX)
  if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_link_libraries(${n} PRIVATE asan ubsan)
  endif ()

  target_link_libraries(${n} PRIVATE m dl pthread)
endif()

set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT ${n})


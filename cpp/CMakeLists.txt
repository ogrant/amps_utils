cmake_minimum_required(VERSION 3.7)

project(knl VERSION 1.0.0 LANGUAGES CXX)

# Build config:
set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_VERBOSE_MAKEFILE OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(fmt CONFIG REQUIRED)
message("fmt version: ${fmt_VERSION}")

find_package(msgpack CONFIG REQUIRED)
message("msgpack version: ${msgpack_VERSION}")

get_cmake_property(_variableNames VARIABLES)
list (SORT _variableNames)
foreach (_variableName ${_variableNames})
   message(STATUS "${_variableName}=${${_variableName}}")
endforeach()

add_executable(
   main
   main.cpp
)

target_link_libraries(
   main
   PUBLIC
   fmt::fmt
   msgpackc
)
cmake_minimum_required(VERSION 3.22.1)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR};${CMAKE_MODULE_PATH};")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(mesh-svg-tool VERSION 0.1.0)

message("Platform toolset (${VCPKG_PLATFORM_TOOLSET})")
message("Target triplet (${VCPKG_TARGET_TRIPLET})")

find_package(argparse CONFIG REQUIRED)
find_package(OpenMesh CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)

set(${PROJECT_NAME}_SOURCE
    ./main.cpp
    )

#target_include_directories(${PROJECT_NAME}
    #PUBLIC
    #${CMAKE_CURRENT_SOURCE_DIR}
    #)

add_executable(${PROJECT_NAME}
    ${${PROJECT_NAME}_SOURCE}
    )

#target_include_directories(${PROJECT_NAME}
    #)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        argparse::argparse
        spdlog::spdlog_header_only
        OpenMeshCore
    )

target_compile_definitions(${PROJECT_NAME} PRIVATE OM_STATIC_BUILD)

set_target_properties(${PROJECT_NAME} PROPERTIES
    LINKER_LANGUAGE CXX
    CXX_STANDARD 20)


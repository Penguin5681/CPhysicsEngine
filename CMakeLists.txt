cmake_minimum_required(VERSION 3.10)
project(CPhysicsEngine)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SFML_STATIC_LIBRARIES ON) # <<< ADD THIS LINE

find_package(SFML 3 COMPONENTS System Window Graphics REQUIRED)

add_executable(CPhysicsEngine
        main.cpp
        vendor/imgui/imgui.cpp
        vendor/imgui/imgui_draw.cpp
        vendor/imgui/imgui_widgets.cpp
        vendor/imgui/imgui_tables.cpp
        vendor/imgui-sfml/imgui-SFML.cpp
)

target_include_directories(CPhysicsEngine PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/vendor/imgui
        ${CMAKE_CURRENT_SOURCE_DIR}/vendor/imgui-sfml
)

target_link_libraries(CPhysicsEngine PRIVATE
        SFML::System
        SFML::Window
        SFML::Graphics
        pthread
)

if (WIN32 AND SFML_STATIC_LIBRARIES)
    target_link_libraries(CPhysicsEngine PRIVATE SFML::Main)
endif ()
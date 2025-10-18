cmake_minimum_required(VERSION 4.0)
project(CPhysicsEngine)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

add_executable(CPhysicsEngine main.cpp
        src/Core/Vector3.h
        src/Dynamics/RigidBody.h
        src/Dynamics/PhysicsWorld.h
        src/Core/Quaternion.h
        src/Core/Matrix3x3.h)

target_link_libraries(CPhysicsEngine sfml-graphics sfml-window sfml-system)
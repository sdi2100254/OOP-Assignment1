cmake_minimum_required(VERSION 3.10)
project(SelfDrivingSimulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(SOURCES
    main.cpp
    GridWorld.cpp
    WorldObject.cpp
    StaticObjects.cpp
    MovingObjects.cpp
    Sensors.cpp
    Vehicle.cpp
    Visualization.cpp
)


add_executable(simulation ${SOURCES})
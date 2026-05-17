cmake_minimum_required(VERSION 3.8)
project(drone_assembly_cell)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../models
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()

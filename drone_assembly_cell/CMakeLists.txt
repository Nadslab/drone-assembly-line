cmake_minimum_required(VERSION 3.8)
project(drone_assembly_cell)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/screw_spawner.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../models
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../worlds
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../meshes
  DESTINATION share/${PROJECT_NAME})

# Expose each mesh inside its model directory so model:// URIs resolve it.
# (model://bottom_plate/bottom_plate.stl → models/bottom_plate/bottom_plate.stl)
foreach(MESH bottom_plate top_plate arm)
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../meshes/${MESH}.stl
    DESTINATION share/${PROJECT_NAME}/models/${MESH})
endforeach()

ament_package()

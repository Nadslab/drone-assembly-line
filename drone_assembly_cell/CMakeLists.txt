cmake_minimum_required(VERSION 3.8)
project(drone_assembly_cell)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz
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

# drone part models — install STL into each model's meshes/ subfolder
# (model://drone_PART/meshes/PART.stl → models/drone_PART/meshes/PART.stl)
foreach(MESH single_arm bottom_plate emax_motor speedybee_stack capacitor vtx camera vtx_antenna rx_antenna tbs_antenna)
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../meshes/${MESH}.stl
    DESTINATION share/${PROJECT_NAME}/models/drone_${MESH}/meshes)
endforeach()

ament_package()

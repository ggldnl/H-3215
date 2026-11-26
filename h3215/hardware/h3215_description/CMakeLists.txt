cmake_minimum_required(VERSION 3.16)
project(h3215_description NONE)

find_package(ament_cmake REQUIRED)

# mjcf/ is the model; urdf/h3215.urdf.xacro and config/joints.yaml are generated from it by
# `python -m h3215_train.description`; meshes/ comes from the CAD
install(DIRECTORY mjcf urdf meshes config DESTINATION share/${PROJECT_NAME})

ament_package()

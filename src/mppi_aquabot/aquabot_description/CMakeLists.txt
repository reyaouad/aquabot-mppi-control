cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(aquabot_description)
find_package(ament_cmake REQUIRED)

# Hooks
ament_environment_hooks("hooks/resource_paths.dsv.in")

install(DIRECTORY urdf mesh DESTINATION share/${PROJECT_NAME})

ament_package()

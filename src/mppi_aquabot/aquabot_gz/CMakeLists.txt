cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(aquabot_gz)

find_package(ament_cmake REQUIRED)

set(CMAKE_CXX_STANDARD 17)

# Gazebo version, use env variable or ROS 2 official pairing for LTS
# https://gazebosim.org/docs/latest/ros_installation/
if(DEFINED ENV{GZ_VERSION})
  set(GZ_VERSION "$ENV{GZ_VERSION}")
elseif(DEFINED ENV{IGN_VERSION})
  set(GZ_VERSION "$ENV{IGN_VERSION}")
elseif("$ENV{ROS_DISTRO}" STRGREATER_EQUAL "lyrical")
  set(GZ_VERSION "jetty")
elseif("$ENV{ROS_DISTRO}" STRGREATER_EQUAL "jazzy")
  set(GZ_VERSION "harmonic")
else()
  set(GZ_VERSION "fortress")
endif()

message(STATUS "Compiling against Gazebo ${GZ_VERSION}")
set(GZ_TARGET_PREFIX gz)

if("${GZ_VERSION}" STRGREATER_EQUAL "jetty")
  set(GZ_MSGS msgs)
  set(GZ_TRANSPORT transport)
  set(GZ_PLUGIN plugin)
  set(GZ_COMMON common)
  set(GZ_SIM sim)
  set(GZ_MATH math)
  set(GZ_RDR rendering)
  set(GZ_SENSORS sensors)
  set(GZ_UTILS utils)
  set(GZ_FUEL fuel_tools)

elseif("${GZ_VERSION}" STREQUAL "harmonic")
  set(GZ_MSGS msgs10)
  set(GZ_TRANSPORT transport13)
  set(GZ_PLUGIN plugin2)
  set(GZ_COMMON common5)
  set(GZ_SIM sim8)
  set(GZ_MATH math7)
  set(GZ_RDR rendering8)
  set(GZ_SENSORS sensors8)
  set(GZ_UTILS utils2)
  set(GZ_FUEL fuel_tools9)

elseif("${GZ_VERSION}" STREQUAL "garden")
  set(GZ_MSGS msgs9)
  set(GZ_TRANSPORT transport12)
  set(GZ_PLUGIN plugin2)
  set(GZ_COMMON common5)
  set(GZ_SIM sim7)
  set(GZ_MATH math7)
  set(GZ_RDR rendering7)
  set(GZ_SENSORS sensors7)
  set(GZ_UTILS utils2)
  set(GZ_FUEL fuel_tools8)

else()
  set(GZ_TARGET_PREFIX ignition)
  set(GZ_MSGS msgs8)
  set(GZ_TRANSPORT transport11)
  set(GZ_PLUGIN plugin1)
  set(GZ_COMMON common4)
  set(GZ_SIM gazebo6)
  set(GZ_MATH math6)
  set(GZ_RDR rendering6)
  set(GZ_SENSORS sensors6)
  set(GZ_UTILS utils1)
  set(GZ_FUEL fuel_tools7)
  add_definitions(-DWITH_IGNITION_FORTRESS)
  add_definitions(-DGZ_UTILS_UNIQUE_IMPL_PTR=IGN_UTILS_UNIQUE_IMPL_PTR)
  add_definitions(-DGZ_PROFILE=IGN_PROFILE)
  add_definitions(-DGZ_ADD_PLUGIN=IGNITION_ADD_PLUGIN)
  add_definitions(-DGZ_ADD_PLUGIN_ALIAS=IGNITION_ADD_PLUGIN_ALIAS)
  add_definitions(-Dgzerr=ignerr)
  add_definitions(-Dgzwarn=ignwarn)
  add_definitions(-Dgzmsg=ignmsg)
  add_definitions(-Dgzdbg=igndbg)
endif()

add_definitions(-DGZ_NS=${GZ_TARGET_PREFIX})
find_package(${GZ_TARGET_PREFIX}-${GZ_MSGS} REQUIRED)
find_package(${GZ_TARGET_PREFIX}-${GZ_TRANSPORT} REQUIRED)
find_package(${GZ_TARGET_PREFIX}-${GZ_PLUGIN} REQUIRED)
find_package(${GZ_TARGET_PREFIX}-${GZ_SIM} REQUIRED)
find_package(${GZ_TARGET_PREFIX}-${GZ_COMMON} REQUIRED)
find_package(${GZ_TARGET_PREFIX}-${GZ_MATH} REQUIRED)
find_package(${GZ_TARGET_PREFIX}-${GZ_RDR} REQUIRED)
find_package(${GZ_TARGET_PREFIX}-${GZ_SENSORS} REQUIRED)
find_package(${GZ_TARGET_PREFIX}-${GZ_UTILS} REQUIRED)
find_package(${GZ_TARGET_PREFIX}-${GZ_FUEL} REQUIRED)

find_package(std_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

#============================================================================
# Hooks
ament_environment_hooks("hooks/resource_paths.dsv.in")


# VRX plugins
add_library(Waves SHARED
  src/vrx/Wavefield.cc
)
target_link_libraries(Waves PUBLIC
  ${GZ_TARGET_PREFIX}-${GZ_COMMON}::core
  ${GZ_TARGET_PREFIX}-${GZ_SIM}::core
  ${GZ_TARGET_PREFIX}-${GZ_MATH}
  Eigen3::Eigen
)
install(
  TARGETS Waves
  DESTINATION lib)

add_library(ScoringPlugin SHARED
  src/vrx/ScoringPlugin.cc
)
target_link_libraries(ScoringPlugin PUBLIC
  ${GZ_TARGET_PREFIX}-${GZ_COMMON}::core
  ${GZ_TARGET_PREFIX}-${GZ_SIM}::core
  ${GZ_TARGET_PREFIX}-${GZ_MATH}
)
install(
  TARGETS ScoringPlugin
  DESTINATION lib)

# Plugins
list(APPEND VRX_GZ_PLUGINS
  PublisherPlugin
  AcousticPingerPlugin
  SimpleHydrodynamics
  Surface
  USVWind
  WaveVisual
)

foreach(PLUGIN ${VRX_GZ_PLUGINS})
  add_library(${PLUGIN} SHARED src/vrx/${PLUGIN}.cc)
  target_link_libraries(${PLUGIN} PUBLIC
    ${GZ_TARGET_PREFIX}-${GZ_SIM}::core
    ${GZ_TARGET_PREFIX}-${GZ_PLUGIN}::core
    ${GZ_TARGET_PREFIX}-${GZ_RDR}::core
    ${GZ_TARGET_PREFIX}-${GZ_SENSORS}::core
    ${GZ_TARGET_PREFIX}-${GZ_UTILS}::core
    #ScoringPlugin
    Waves
    Eigen3::Eigen
  )
endforeach()

install(
  TARGETS ${VRX_GZ_PLUGINS}
  DESTINATION lib)


# WindturbinesInspectionScoringPlugin scoring plugin
add_library(WindturbinesInspectionScoringPlugin SHARED
  src/WindturbinesInspectionScoringPlugin.cc
  src/vrx/WaypointMarkers.cc
)
target_link_libraries(WindturbinesInspectionScoringPlugin PUBLIC
  ${GZ_TARGET_PREFIX}-${GZ_COMMON}::core
  ${GZ_TARGET_PREFIX}-${GZ_SIM}::core
  ${GZ_TARGET_PREFIX}-${GZ_MATH}
  ScoringPlugin
)
install(
  TARGETS WindturbinesInspectionScoringPlugin
  DESTINATION lib)


#============================================================================
# Resources
install(DIRECTORY
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()

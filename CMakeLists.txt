cmake_minimum_required(VERSION 3.8)
project(px4_commander)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- Required packages ----------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(px4_msgs REQUIRED)


# ---- Include directories --------------------------------------
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


# ---- Executables ---------------------------------------------- 

# ============================
#   px4_commander_server
# ============================

add_executable(px4_commander_server
  src/px4_commander_server_node.cpp
)

target_include_directories(px4_commander_server
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(px4_commander_server
  rclcpp
  std_srvs
  px4_msgs
)

install(
  TARGETS
    px4_commander_server
  DESTINATION 
    lib/${PROJECT_NAME}
)


# ============================
#   example01 (cpp)
# ============================

add_executable(example01
  examples/cpp/example01.cpp
)

target_include_directories(example01
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(example01
  rclcpp
  std_srvs
  px4_msgs
)

target_link_libraries(example01
  px4_commander_client
)

install(
  TARGETS
    example01
  DESTINATION 
    lib/${PROJECT_NAME}
)

# ============================
#   example01 (py)
# ============================

install(
  PROGRAMS
    examples/python/example01.py
  DESTINATION 
    lib/${PROJECT_NAME}
)


# ---- APIs ----------------------------------------------------- 

# ============================
#   px4_commander_client (cpp)
# ============================

# Declare shared library
add_library(px4_commander_client 
  SHARED
    src/px4_commander_client.cpp
)

# Build (to create .so)
target_include_directories(px4_commander_client 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(px4_commander_client
  rclcpp
  std_srvs
)

# Install headers
install(
  DIRECTORY 
    include/
  DESTINATION 
    include
)

# Install shared library
install(
  TARGETS 
    px4_commander_client
  EXPORT 
    px4_commander_clientTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(px4_commander_clientTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_srvs)


# ============================
#   px4_commander_client (py)
# ============================

ament_python_install_package(px4_commander
  PACKAGE_DIR px4_commander
)


# ---- Launch / Config ------------------------------------------ 

install(
  DIRECTORY 
    launch
    config
  DESTINATION 
    share/${PROJECT_NAME}
)


ament_package()

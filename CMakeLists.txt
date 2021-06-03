cmake_minimum_required(VERSION 2.8.3)
project(driver_quad_encoder)

# Find catkin and package dependencies.
find_package(catkin REQUIRED
  roscpp
  sensor_msgs_ext)

# Find system dependencies.
find_library(pigpiod pigpiod_if2)

# Create catkin package.
catkin_package(CATKIN_DEPENDS roscpp sensor_msgs_ext)

# Set up include directories.
include_directories(${catkin_INCLUDE_DIRS})

# Build for RPi Interface
if(pigpiod)
    message(STATUS "PIGPIO found. Building rpi_node.")
    # Build node.
    add_executable(${PROJECT_NAME}_rpi_node src/main_rpi.cpp src/ros_node.cpp src/rpi_driver.cpp src/driver.cpp)
    add_dependencies(${PROJECT_NAME}_rpi_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    set_target_properties(${PROJECT_NAME}_rpi_node PROPERTIES OUTPUT_NAME rpi_node PREFIX "")
    target_link_libraries(${PROJECT_NAME}_rpi_node
      ${catkin_LIBRARIES}
      ${pigpiod})
    # Install node.
    install(TARGETS ${PROJECT_NAME}_rpi_node
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
else()
    message(WARNING "PIGPIO not found. Skipping rpi_node.")
endif()
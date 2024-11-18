cmake_minimum_required(VERSION 3.8)
project(rviz_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(multi_map_server REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Include directories
include_directories(include)

# Qt-specific configurations
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Source and header files
set(SOURCE_FILES
  src/pose_tool.cpp
  # src/goal_tool.cpp
  # src/probmap_display.cpp
  # src/aerialmap_display.cpp   
  # src/multi_probmap_display.cpp
)

set(HEADER_FILES
  src/pose_tool.h
  # src/goal_tool.h
  # src/probmap_display.h
  # src/aerialmap_display.h
  # src/multi_probmap_display.h
)

# Add the library
add_library(${PROJECT_NAME} SHARED
  ${SOURCE_FILES}
  ${HEADER_FILES}
)


ament_target_dependencies(${PROJECT_NAME} rviz_common)

# Link libraries
target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  # rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  rviz_default_plugins::rviz_default_plugins
  multi_map_server
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install plugin description and other resources
install(FILES
  plugin_description.xml
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY media/
  DESTINATION share/${PROJECT_NAME}/media
)

install(DIRECTORY icons/
  DESTINATION share/${PROJECT_NAME}/icons
)


ament_package()

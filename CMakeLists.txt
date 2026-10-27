cmake_minimum_required(VERSION 3.8)
project(nav2_error_compensated_pure_pursuit_controller)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(nav2_regulated_pure_pursuit_controller REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

nav2_package()
set(CMAKE_CXX_STANDARD 20)

include_directories(
  include
)

set(dependencies
  angles
  rclcpp
  rcl_interfaces
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  std_msgs
  nav2_util
  nav2_ros_common
  nav2_core
  nav2_regulated_pure_pursuit_controller
  tf2
  tf2_geometry_msgs
)

set(dependency_targets)
foreach(dependency IN LISTS dependencies)
  list(APPEND dependency_targets ${${dependency}_TARGETS})
endforeach()

set(library_name nav2_error_compensated_pure_pursuit_controller)

add_library(${library_name} SHARED
  src/error_compensated_pure_pursuit_controller.cpp
  src/path_handler.cpp
)

target_link_libraries(${library_name} ${dependency_targets})

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_core nav2_error_compensated_pure_pursuit_controller.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ecpp_math
    test/test_ecpp_math.cpp
  )
  if(TARGET test_ecpp_math)
    target_link_libraries(test_ecpp_math ${dependency_targets})
    target_include_directories(test_ecpp_math PUBLIC
      include
    )
  endif()

  ament_add_gtest(test_arc_length_lookahead
    test/test_arc_length_lookahead.cpp
  )
  if(TARGET test_arc_length_lookahead)
    target_link_libraries(test_arc_length_lookahead ${dependency_targets})
    target_include_directories(test_arc_length_lookahead PUBLIC
      include
    )
  endif()

  ament_add_gtest(test_dwpp_functions
    test/test_dwpp_functions.cpp
  )
  if(TARGET test_dwpp_functions)
    target_link_libraries(test_dwpp_functions ${dependency_targets})
    target_include_directories(test_dwpp_functions PUBLIC
      include
    )
  endif()

  ament_add_gtest(test_controller_options
    test/test_controller_options.cpp
  )
  if(TARGET test_controller_options)
    target_link_libraries(test_controller_options ${dependency_targets})
    target_include_directories(test_controller_options PUBLIC
      include
    )
    target_link_libraries(test_controller_options
      ${library_name}
    )
  endif()

  ament_add_gtest(test_path_handler
    test/test_path_handler.cpp
  )
  if(TARGET test_path_handler)
    target_link_libraries(test_path_handler ${dependency_targets})
    target_include_directories(test_path_handler PUBLIC
      include
    )
    target_link_libraries(test_path_handler
      ${library_name}
    )
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()

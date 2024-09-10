cmake_minimum_required(VERSION 3.5)
project(robotic_planning)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    plansys2_msgs
    plansys2_executor
)

add_executable(empty_box src/empty_box.cpp)
ament_target_dependencies(empty_box ${dependencies})

add_executable(fill_box src/fill_box.cpp)
ament_target_dependencies(fill_box ${dependencies})

add_executable(load_carrier src/load_carrier.cpp)
ament_target_dependencies(load_carrier ${dependencies})

add_executable(move_with_carrier src/move_with_carrier.cpp)
ament_target_dependencies(move_with_carrier ${dependencies})


install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(TARGETS
  empty_box
  fill_box
  load_carrier
  move_with_carrier
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()
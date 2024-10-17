cmake_minimum_required(VERSION 3.8)
project(judge_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(Boost COMPONENTS locale date_time filesystem iostreams REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED)

#find source
file(GLOB_RECURSE src
    "src/**"
)

ament_auto_add_executable(${PROJECT_NAME} ${src})
set(link ${link} Boost::locale Boost::date_time Boost::filesystem Boost::iostreams)
target_link_libraries(${PROJECT_NAME} ${link})

# rclcpp_components_register_node(${PROJECT_NAME}
#   PLUGIN nn_detector::DetectorNode
#   EXECUTABLE ${PROJECT_NAME}_node
# )

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

ament_auto_package()

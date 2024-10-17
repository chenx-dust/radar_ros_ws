cmake_minimum_required(VERSION 3.22)
project(pc_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()


find_package(Boost COMPONENTS date_time filesystem iostreams REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED)

file(GLOB_RECURSE SRC_FILES src/**)

ament_auto_add_library(${PROJECT_NAME} SHARED ${SRC_FILES})


target_link_libraries(${PROJECT_NAME}
  Open3D::Open3D
  spdlog::spdlog_header_only
  yaml-cpp
  Boost::boost
  Boost::date_time
  Boost::filesystem
  Boost::iostreams
  nlohmann_json::nlohmann_json
  TBB::tbb
  Eigen3::Eigen
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN pc_detector::DetectorNode
  EXECUTABLE ${PROJECT_NAME}_node
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE)

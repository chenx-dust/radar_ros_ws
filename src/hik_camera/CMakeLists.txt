cmake_minimum_required(VERSION 3.8)
project(hik_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-O3)
set(CMAKE_CXX_STANDARD 17)

# OpenCV
find_package(OpenCV 4 REQUIRED)
set(link ${OpenCV_LIBS} ${link})

# find ament_cmake
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# find src
ament_auto_add_library(${PROJECT_NAME} SHARED src/hik_camera.cpp)
# ament_auto_add_executable(${PROJECT_NAME} ${HkCamera_src})
target_link_libraries(${PROJECT_NAME} ${link})

# find dependencies
target_include_directories(${PROJECT_NAME} PUBLIC /opt/MVS/include)
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  message("aarch64 environ, link MVS to /opt/MVS/lib/aarch64")
  target_link_directories(${PROJECT_NAME} PUBLIC /opt/MVS/lib/aarch64)
else()
  target_link_directories(${PROJECT_NAME} PUBLIC /opt/MVS/lib/64)
endif()
target_link_libraries(${PROJECT_NAME}
  MvCameraControl
  MvUsb3vTL
  MVGigEVisionSDK
  MediaProcess
  FormatConversion
  MVRender
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN hik_camera::HikCameraNode
  EXECUTABLE ${PROJECT_NAME}_node
)


ament_auto_add_executable(info_pub src/info_pub.cpp)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


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

ament_auto_package(
  INSTALL_TO_SHARE
  config
)

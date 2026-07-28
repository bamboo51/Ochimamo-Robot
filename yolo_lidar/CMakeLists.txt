cmake_minimum_required(VERSION 3.8)
project(yolo_lidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── ROS2 dependencies ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# ── OpenCV (for Hough transform, ArUco, image conversion) ─────────────────────
find_package(OpenCV REQUIRED)

# ── MNN (for YOLO inference) ──────────────────────────────────────────────────
find_path(MNN_INCLUDE_DIR
  NAMES MNN/Interpreter.hpp
  HINTS /usr/local/include /usr/include
)
find_library(MNN_LIB
  NAMES MNN
  HINTS /usr/local/lib /usr/lib
)

if(NOT MNN_INCLUDE_DIR OR NOT MNN_LIB)
  message(FATAL_ERROR
    "MNN not found. Build and install MNN C++ SDK first:\n"
    "  git clone https://github.com/alibaba/MNN && cd MNN && mkdir build && cd build\n"
    "  cmake .. -DMNN_BUILD_SHARED_LIBS=ON -DMNN_BUILD_CONVERTER=OFF\n"
    "  make -j4 && sudo make install"
  )
endif()

message(STATUS "MNN include: ${MNN_INCLUDE_DIR}")
message(STATUS "MNN lib:     ${MNN_LIB}")

# ── Executable ────────────────────────────────────────────────────────────────
add_executable(people_mapper_node
  src/main.cpp
  src/lidar_system.cpp
  src/vision_system.cpp
  src/people_mapper_node.cpp
)

target_include_directories(people_mapper_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${MNN_INCLUDE_DIR}
)

ament_target_dependencies(people_mapper_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_transport
)

target_link_libraries(people_mapper_node
  ${OpenCV_LIBS}
  ${MNN_LIB}
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS people_mapper_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

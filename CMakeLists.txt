cmake_minimum_required(VERSION 3.5)
project(orb_slam3_ros)

if(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-w)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_common REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
# find_package(rosidl_default_generators REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_common
  image_transport
  tf2_geometry_msgs
  tf2_ros
)

find_package(OpenCV 4.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 3.0 QUIET)
   if(NOT OpenCV_FOUND)
      find_package(OpenCV 2.4.3 QUIET)
      if(NOT OpenCV_FOUND)
         message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
     endif()
   endif()
endif()

find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/SaveMap.srv"
#   DEPENDENCIES builtin_interfaces std_msgs
# )

include_directories(
  ${PROJECT_SOURCE_DIR}/orb_slam3
  ${PROJECT_SOURCE_DIR}/orb_slam3/include
  ${PROJECT_SOURCE_DIR}/orb_slam3/include/CameraModels
  ${PROJECT_SOURCE_DIR}/ros/include
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  include
)

include(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/CMakeLists.txt)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/orb_slam3/lib)

add_library(orb_slam3_ros_core SHARED
orb_slam3/src/Atlas.cc
orb_slam3/src/Converter.cc
orb_slam3/src/Frame.cc
orb_slam3/src/FrameDrawer.cc
orb_slam3/src/G2oTypes.cc
orb_slam3/src/ImuTypes.cc
orb_slam3/src/Initializer.cc
orb_slam3/src/KeyFrame.cc
orb_slam3/src/KeyFrameDatabase.cc
orb_slam3/src/LocalMapping.cc
orb_slam3/src/LoopClosing.cc
orb_slam3/src/Converter.cc
orb_slam3/src/Map.cc
orb_slam3/src/MapDrawer.cc
orb_slam3/src/MapPoint.cc
orb_slam3/src/MLPnPsolver.cpp
orb_slam3/src/OptimizableTypes.cpp
orb_slam3/src/Optimizer.cc
orb_slam3/src/ORBextractor.cc
orb_slam3/src/ORBmatcher.cc
orb_slam3/src/PnPsolver.cc
orb_slam3/src/Sim3Solver.cc
orb_slam3/src/System.cc
orb_slam3/src/Tracking.cc
orb_slam3/src/TwoViewReconstruction.cc
orb_slam3/src/Viewer.cc
orb_slam3/src/CameraModels/KannalaBrandt8.cpp
orb_slam3/src/CameraModels/Pinhole.cpp
)

ament_target_dependencies(orb_slam3_ros_core
  rclcpp
  std_msgs
  sensor_msgs
)
target_link_libraries(orb_slam3_ros_core
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  DBoW2
  g2o
)

# map serialization addition - library boost serialization 
message(STATUS "Compile With map save/load function")
find_library(BOOST_SERIALIZATION boost_serialization)
if (NOT BOOST_SERIALIZATION)
    message(FATAL_ERROR "Can't find libboost_serialization")
endif()
target_link_libraries(orb_slam3_ros_core ${BOOST_SERIALIZATION})

#Mono
add_executable(${PROJECT_NAME}_mono
  ros/src/MonoNode.cc
  ros/src/Node.cc
)
ament_target_dependencies(${PROJECT_NAME}_mono
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}_mono
  orb_slam3_ros_core
  ${OpenCV_LIBS}
)

# rosidl_target_interfaces(${PROJECT_NAME}_mono ${PROJECT_NAME} "rosidl_typesupport_cpp")

#Stereo
add_executable (${PROJECT_NAME}_stereo
  ros/src/StereoNode.cc
  ros/src/Node.cc
)
ament_target_dependencies(${PROJECT_NAME}_stereo
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}_stereo
  orb_slam3_ros_core
  ${OpenCV_LIBS}
)

# rosidl_target_interfaces(${PROJECT_NAME}_stereo ${PROJECT_NAME} "rosidl_typesupport_cpp")

#RGBD
add_executable(${PROJECT_NAME}_rgbd
  ros/src/RGBDNode.cc
  ros/src/Node.cc
)
ament_target_dependencies(${PROJECT_NAME}_rgbd
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}_rgbd
  orb_slam3_ros_core
  ${OpenCV_LIBS}
)

# rosidl_target_interfaces(${PROJECT_NAME}_rgbd ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS ${PROJECT_NAME}_mono ${PROJECT_NAME}_stereo ${PROJECT_NAME}_rgbd
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS orb_slam3_ros_core DBoW2 g2o
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY ros/launch/
  DESTINATION share/${PROJECT_NAME}/ros/launch
)

install(DIRECTORY ros/config/
  DESTINATION share/${PROJECT_NAME}/ros/config
)

install(DIRECTORY orb_slam3/Vocabulary/
  DESTINATION share/${PROJECT_NAME}/orb_slam3/Vocabulary
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  ${dependencies}
  # rosidl_default_runtime
  DBoW2
  g2o
)
ament_export_libraries(orb_slam3_ros_core libDBoW2 libg2o)
ament_package()

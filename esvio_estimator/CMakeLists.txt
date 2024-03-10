cmake_minimum_required(VERSION 2.8.3)
project(esvio_estimator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    camera_model
    )

find_package(OpenCV REQUIRED)

find_package(Ceres REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package()

add_executable(stereo_esvio_estimator
    src/stereo_estimator_node.cpp
    src/parameters.cpp
    src/estimator.cpp
    src/feature_manager.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/projection_factor.cpp
    src/factor/projection_td_factor.cpp
    src/factor/marginalization_factor.cpp
    src/factor/projectionOneFrameTwoCamFactor.cpp #
    src/factor/projectionTwoFrameOneCamFactor.cpp#
    src/factor/projectionTwoFrameTwoCamFactor.cpp #
    src/utility/utility.cpp
    src/utility/visualization.cpp
    src/utility/CameraPoseVisualization.cpp
    src/initial/solve_5pts.cpp
    src/initial/solve_opt.cpp #
    src/initial/translate_factor.cpp #
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp
    )

  target_link_libraries(stereo_esvio_estimator 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS} 
  ${CERES_LIBRARIES} 
  )
  
  add_executable(stereo_esio_estimator
  src/esio_estimator_node.cpp
  src/parameters.cpp
  src/estimator.cpp
  src/feature_manager.cpp
  src/factor/pose_local_parameterization.cpp
  src/factor/projection_factor.cpp
  src/factor/projection_td_factor.cpp
  src/factor/marginalization_factor.cpp
  src/factor/projectionOneFrameTwoCamFactor.cpp #
  src/factor/projectionTwoFrameOneCamFactor.cpp#
  src/factor/projectionTwoFrameTwoCamFactor.cpp #
  src/utility/utility.cpp
  src/utility/visualization.cpp
  src/utility/CameraPoseVisualization.cpp
  src/initial/solve_5pts.cpp
  src/initial/solve_opt.cpp #
  src/initial/translate_factor.cpp #
  src/initial/initial_aligment.cpp
  src/initial/initial_sfm.cpp
  src/initial/initial_ex_rotation.cpp
  )

target_link_libraries(stereo_esio_estimator 
${catkin_LIBRARIES} 
${OpenCV_LIBS} 
${CERES_LIBRARIES} 
) 



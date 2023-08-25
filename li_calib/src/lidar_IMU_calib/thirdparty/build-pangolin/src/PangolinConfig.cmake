# Compute paths
get_filename_component( PROJECT_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH )
SET( Pangolin_INCLUDE_DIRS "/home/jian/licalib_ws/src/lidar_IMU_calib/thirdparty/Pangolin/include;/home/jian/licalib_ws/src/lidar_IMU_calib/thirdparty/build-pangolin/src/include;/usr/include;/usr/include;/usr/include;/usr/include/eigen3" )
SET( Pangolin_INCLUDE_DIR  "/home/jian/licalib_ws/src/lidar_IMU_calib/thirdparty/Pangolin/include;/home/jian/licalib_ws/src/lidar_IMU_calib/thirdparty/build-pangolin/src/include;/usr/include;/usr/include;/usr/include;/usr/include/eigen3" )

# Library dependencies (contains definitions for IMPORTED targets)
if( NOT TARGET pangolin AND NOT Pangolin_BINARY_DIR )
  include( "${PROJECT_CMAKE_DIR}/PangolinTargets.cmake" )
  
endif()

SET( Pangolin_LIBRARIES    pangolin )
SET( Pangolin_LIBRARY      pangolin )
SET( Pangolin_CMAKEMODULES /home/jian/licalib_ws/src/lidar_IMU_calib/thirdparty/Pangolin/src/../CMakeModules )

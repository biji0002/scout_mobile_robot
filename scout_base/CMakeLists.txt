cmake_minimum_required(VERSION 2.8.3)
project(scout_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# cmake module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS    
    roslaunch 
    roslint 
    roscpp 
    sensor_msgs
    std_msgs
    geometry_msgs
    scout_msgs
    scout_sdk
    tf2 
    tf2_ros
    message_generation
    rospy
)
    
find_package(LCM REQUIRED)
find_package(OpenCV REQUIRED)





# find_package(Boost REQUIRED COMPONENTS chrono)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES scout_messenger
  CATKIN_DEPENDS scout_msgs roscpp sensor_msgs scout_sdk scout_msgs
#   DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${LCM_INCLUDE_DIRS}
)

add_library(scout_messenger STATIC src/scout_messenger.cpp)
target_link_libraries(scout_messenger ${catkin_LIBRARIES})
set_property(TARGET scout_messenger PROPERTY POSITION_INDEPENDENT_CODE ON)

add_executable(scout_base_node src/scout_base_node.cpp)
target_link_libraries(scout_base_node scout_messenger ${catkin_LIBRARIES})

add_executable(sensorhub_node src/sensorhub_node.cpp)
target_link_libraries(sensorhub_node ${catkin_LIBRARIES} ${LCM_LIBRARIES})

add_executable(scout_csi_camera src/scout_csi_camera.cpp)
target_link_libraries(scout_csi_camera ${catkin_LIBRARIES} ${LCM_LIBRARIES} ${OpenCV_LIBS})

#add_executable(test_ptz_ctrl tests/test_ptz_ctrl.cpp)
#target_link_libraries(test_ptz_ctrl ${catkin_LIBRARIES} ${LCM_LIBRARIES} ${OpenCV_LIBS})


add_executable(ptz_ctrl_publisher tests/ptz_ctrl_publisher.cpp)
target_link_libraries(ptz_ctrl_publisher ${catkin_LIBRARIES} ${LCM_LIBRARIES} ${OpenCV_LIBS})

add_executable(ptz_cam_srv_client src/ptz_cam_srv_client.cpp)
target_link_libraries(ptz_cam_srv_client ${catkin_LIBRARIES} ${LCM_LIBRARIES} ${OpenCV_LIBS})

add_executable(ptz_service_call_node tests/ptz_service_call_node.cpp)
target_link_libraries(ptz_service_call_node ${catkin_LIBRARIES} ${LCM_LIBRARIES} ${OpenCV_LIBS})

#add_executable(pose_ptz_cam src/pose_ptz_cam.cpp)
#target_link_libraries(pose_ptz_cam ${catkin_LIBRARIES} ${LCM_LIBRARIES} ${OpenCV_LIBS})


#############
## Install ##
#############

roslaunch_add_file_check(launch)

install(TARGETS scout_messenger scout_base_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

  install(DIRECTORY include/lcmtypes
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(DIRECTORY launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

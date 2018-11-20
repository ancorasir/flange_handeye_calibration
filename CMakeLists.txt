cmake_minimum_required(VERSION 2.8.12)
project(phoxi_camera)

find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.8 REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    roslib
    rospy
    std_msgs
    sensor_msgs
    pcl_ros
    pcl_conversions
    #pcl_visualization
    dynamic_reconfigure
    diagnostic_updater
    geometry_msgs
    eigen_conversions
    cv_bridge
)

catkin_python_setup()



include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PHOXI_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_message_files(
  FILES
    PhoXiSize.msg
)

add_service_files(
  FILES
    Deprecated/Empty.srv        #depricated
    Deprecated/IsConnected.srv  #depricated
    Deprecated/IsAcquiring.srv  #depricated
    ConnectCamera.srv
    TriggerImage.srv
    GetDeviceList.srv
    GetBool.srv
    GetFrame.srv
    GetHardwareIdentification.srv
    GetSupportedCapturingModes.srv
    SaveFrame.srv
    SetCoordinatesSpace.srv
    SetTransformationMatrix.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    sensor_msgs
)

generate_dynamic_reconfigure_options(
  cfg/phoxi_camera.cfg
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    std_msgs
    sensor_msgs 
)

add_compile_options(-w)
add_compile_options(-std=c++11)
add_compile_options(-fpermissive)
add_compile_options(-pthread)

add_library(
  ${PROJECT_NAME}_PhoXi_Interface
  src/PhoXiInterface.cpp
)

add_library(
  ${PROJECT_NAME}_Ros_Interface
  src/RosInterface.cpp
)

add_executable(
  ${PROJECT_NAME}
  src/phoxi_camera_node.cpp
  src/PhoXiInterface.cpp
  src/RosInterface.cpp
)

add_dependencies(
  ${PROJECT_NAME}_Ros_Interface
  ${PROJECT_NAME}/_PhoXi_Interfece
  ${PROJECT_NAME}_gencfgv
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_gencfgv
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}_PhoXi_Interface
  ${PHOXI_LIBRARY}
  rt
  ${Boost_LIBRARIES}
)
target_link_libraries(
  ${PROJECT_NAME}_Ros_Interface
  ${PROJECT_NAME}_PhoXi_Interface
  ${catkin_LIBRARIES}

)

target_link_libraries(
  ${PROJECT_NAME}
  debug ${catkin_LIBRARIES}
  optimized ${catkin_LIBRARIES}
  ${PHOXI_LIBRARY}
  ${Boost_LIBRARIES}
  rt
)

#### New CPP Files ######
set (SOURCE_FILES 
     src/api_template/phoxi_camera_test_cpp.cpp
     src/api_template/phoxi_camera_test_cpp.h  
)

add_executable(phoxi_camera_test src/api_template/phoxi_camera_test_cpp.cpp)
target_link_libraries(phoxi_camera_test ${catkin_LIBRARIES})
add_dependencies(phoxi_camera_test ${catkin_EXPORTED_TARGETS})

set (SOURCE_FILES 
     src/cal_dev/phoxi_camera_cal_camShooter.cpp
     src/cal_dev/phoxi_camera_cal_lib.h  
)
add_executable(phoxi_camera_cal_camShooter src/cal_dev/phoxi_camera_cal_camShooter.cpp)
target_link_libraries(phoxi_camera_cal_camShooter ${catkin_LIBRARIES})
add_dependencies(phoxi_camera_cal_camShooter ${catkin_EXPORTED_TARGETS})

set (SOURCE_FILES 
     src/cal_dev/phoxi_camera_cal_listener.cpp
     src/cal_dev/phoxi_camera_cal_lib.h  
)
add_executable(phoxi_camera_cal_listener src/cal_dev/phoxi_camera_cal_listener.cpp)
target_link_libraries(phoxi_camera_cal_listener ${catkin_LIBRARIES})
add_dependencies(phoxi_camera_cal_listener ${catkin_EXPORTED_TARGETS})

set (SOURCE_FILES 
     src/cal_dev/phoxi_camera_cal_test.cpp
     src/cal_dev/phoxi_camera_cal_lib.h  
)
add_executable(phoxi_camera_cal_test src/cal_dev/phoxi_camera_cal_test.cpp)
target_link_libraries(phoxi_camera_cal_test ${catkin_LIBRARIES})
add_dependencies(phoxi_camera_cal_test ${catkin_EXPORTED_TARGETS})

#### New CPP Files End ######

install(
  PROGRAMS
    src/phoxi_camera_example.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
    ${PHOXI_LIB_RELEASE_PER_COMPILER}.${PHO_SOFTWARE_VERSION}
  DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(PROGRAMS
  test/interfaces/ros_utils.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/interfaces/launch/test_phoxi_camera_interfaces_exist.test)
    add_rostest(test/interfaces/launch/test_phoxi_camera_ros_interfaces.test)

    add_rostest_gtest(${PROJECT_NAME}_unittest
            test/gtest/launch/test_phoxi_interface_class.test
            test/gtest/test_phoxi_interface.cpp)

    target_link_libraries(${PROJECT_NAME}_unittest
            ${catkin_LIBRARIES}
            ${PROJECT_NAME}_Ros_Interface
            ${PROJECT_NAME}_PhoXi_Interface)
endif()

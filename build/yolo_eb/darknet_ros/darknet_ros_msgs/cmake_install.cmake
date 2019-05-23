# Install script for directory: /home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/erlendb/Programmering/Master/test_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/darknet_ros_msgs_eb/msg" TYPE FILE FILES
    "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg"
    "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/darknet_ros_msgs_eb/action" TYPE FILE FILES "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/action/CheckForObjects.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/darknet_ros_msgs_eb/msg" TYPE FILE FILES
    "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsAction.msg"
    "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionGoal.msg"
    "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionResult.msg"
    "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionFeedback.msg"
    "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsGoal.msg"
    "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsResult.msg"
    "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/darknet_ros_msgs_eb/cmake" TYPE FILE FILES "/home/erlendb/Programmering/Master/test_ws/build/yolo_eb/darknet_ros/darknet_ros_msgs/catkin_generated/installspace/darknet_ros_msgs_eb-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/erlendb/Programmering/Master/test_ws/devel/include/darknet_ros_msgs_eb")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/erlendb/Programmering/Master/test_ws/devel/share/roseus/ros/darknet_ros_msgs_eb")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/erlendb/Programmering/Master/test_ws/devel/share/common-lisp/ros/darknet_ros_msgs_eb")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/erlendb/Programmering/Master/test_ws/devel/share/gennodejs/ros/darknet_ros_msgs_eb")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/home/erlendb/anaconda2/bin/python" -m compileall "/home/erlendb/Programmering/Master/test_ws/devel/lib/python2.7/dist-packages/darknet_ros_msgs_eb")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/erlendb/Programmering/Master/test_ws/devel/lib/python2.7/dist-packages/darknet_ros_msgs_eb")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/erlendb/Programmering/Master/test_ws/build/yolo_eb/darknet_ros/darknet_ros_msgs/catkin_generated/installspace/darknet_ros_msgs_eb.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/darknet_ros_msgs_eb/cmake" TYPE FILE FILES "/home/erlendb/Programmering/Master/test_ws/build/yolo_eb/darknet_ros/darknet_ros_msgs/catkin_generated/installspace/darknet_ros_msgs_eb-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/darknet_ros_msgs_eb/cmake" TYPE FILE FILES
    "/home/erlendb/Programmering/Master/test_ws/build/yolo_eb/darknet_ros/darknet_ros_msgs/catkin_generated/installspace/darknet_ros_msgs_ebConfig.cmake"
    "/home/erlendb/Programmering/Master/test_ws/build/yolo_eb/darknet_ros/darknet_ros_msgs/catkin_generated/installspace/darknet_ros_msgs_ebConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/darknet_ros_msgs_eb" TYPE FILE FILES "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/package.xml")
endif()


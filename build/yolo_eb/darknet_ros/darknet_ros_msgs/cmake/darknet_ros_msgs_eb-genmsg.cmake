# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "darknet_ros_msgs_eb: 9 messages, 0 services")

set(MSG_I_FLAGS "-Idarknet_ros_msgs_eb:/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg;-Idarknet_ros_msgs_eb:/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(darknet_ros_msgs_eb_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionResult.msg" NAME_WE)
add_custom_target(_darknet_ros_msgs_eb_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "darknet_ros_msgs_eb" "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionResult.msg" "std_msgs/Header:darknet_ros_msgs_eb/CheckForObjectsResult:darknet_ros_msgs_eb/BoundingBox:actionlib_msgs/GoalID:darknet_ros_msgs_eb/BoundingBoxes:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsAction.msg" NAME_WE)
add_custom_target(_darknet_ros_msgs_eb_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "darknet_ros_msgs_eb" "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsAction.msg" "darknet_ros_msgs_eb/BoundingBox:std_msgs/Header:darknet_ros_msgs_eb/CheckForObjectsActionGoal:darknet_ros_msgs_eb/CheckForObjectsFeedback:darknet_ros_msgs_eb/CheckForObjectsActionFeedback:darknet_ros_msgs_eb/CheckForObjectsResult:darknet_ros_msgs_eb/CheckForObjectsGoal:sensor_msgs/Image:darknet_ros_msgs_eb/CheckForObjectsActionResult:actionlib_msgs/GoalID:darknet_ros_msgs_eb/BoundingBoxes:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionFeedback.msg" NAME_WE)
add_custom_target(_darknet_ros_msgs_eb_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "darknet_ros_msgs_eb" "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionFeedback.msg" "darknet_ros_msgs_eb/CheckForObjectsFeedback:actionlib_msgs/GoalID:std_msgs/Header:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg" NAME_WE)
add_custom_target(_darknet_ros_msgs_eb_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "darknet_ros_msgs_eb" "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg" "darknet_ros_msgs_eb/BoundingBox:std_msgs/Header"
)

get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsResult.msg" NAME_WE)
add_custom_target(_darknet_ros_msgs_eb_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "darknet_ros_msgs_eb" "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsResult.msg" "darknet_ros_msgs_eb/BoundingBox:darknet_ros_msgs_eb/BoundingBoxes:std_msgs/Header"
)

get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg" NAME_WE)
add_custom_target(_darknet_ros_msgs_eb_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "darknet_ros_msgs_eb" "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg" ""
)

get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsFeedback.msg" NAME_WE)
add_custom_target(_darknet_ros_msgs_eb_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "darknet_ros_msgs_eb" "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsFeedback.msg" ""
)

get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionGoal.msg" NAME_WE)
add_custom_target(_darknet_ros_msgs_eb_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "darknet_ros_msgs_eb" "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionGoal.msg" "darknet_ros_msgs_eb/CheckForObjectsGoal:sensor_msgs/Image:actionlib_msgs/GoalID:std_msgs/Header"
)

get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsGoal.msg" NAME_WE)
add_custom_target(_darknet_ros_msgs_eb_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "darknet_ros_msgs_eb" "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsGoal.msg" "sensor_msgs/Image:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsResult.msg;/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_cpp(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_cpp(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_cpp(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsAction.msg"
  "${MSG_I_FLAGS}"
  "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionGoal.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsFeedback.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionFeedback.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsResult.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsGoal.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_cpp(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsResult.msg"
  "${MSG_I_FLAGS}"
  "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg;/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_cpp(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_cpp(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_cpp(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsGoal.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_cpp(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/darknet_ros_msgs_eb
)

### Generating Services

### Generating Module File
_generate_module_cpp(darknet_ros_msgs_eb
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/darknet_ros_msgs_eb
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(darknet_ros_msgs_eb_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(darknet_ros_msgs_eb_generate_messages darknet_ros_msgs_eb_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionResult.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_cpp _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsAction.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_cpp _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionFeedback.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_cpp _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_cpp _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsResult.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_cpp _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_cpp _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsFeedback.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_cpp _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionGoal.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_cpp _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsGoal.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_cpp _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(darknet_ros_msgs_eb_gencpp)
add_dependencies(darknet_ros_msgs_eb_gencpp darknet_ros_msgs_eb_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS darknet_ros_msgs_eb_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsResult.msg;/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_eus(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_eus(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_eus(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsAction.msg"
  "${MSG_I_FLAGS}"
  "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionGoal.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsFeedback.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionFeedback.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsResult.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsGoal.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_eus(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsResult.msg"
  "${MSG_I_FLAGS}"
  "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg;/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_eus(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_eus(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_eus(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsGoal.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_eus(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/darknet_ros_msgs_eb
)

### Generating Services

### Generating Module File
_generate_module_eus(darknet_ros_msgs_eb
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/darknet_ros_msgs_eb
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(darknet_ros_msgs_eb_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(darknet_ros_msgs_eb_generate_messages darknet_ros_msgs_eb_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionResult.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_eus _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsAction.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_eus _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionFeedback.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_eus _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_eus _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsResult.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_eus _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_eus _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsFeedback.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_eus _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionGoal.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_eus _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsGoal.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_eus _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(darknet_ros_msgs_eb_geneus)
add_dependencies(darknet_ros_msgs_eb_geneus darknet_ros_msgs_eb_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS darknet_ros_msgs_eb_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsResult.msg;/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_lisp(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_lisp(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_lisp(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsAction.msg"
  "${MSG_I_FLAGS}"
  "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionGoal.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsFeedback.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionFeedback.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsResult.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsGoal.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_lisp(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsResult.msg"
  "${MSG_I_FLAGS}"
  "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg;/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_lisp(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_lisp(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_lisp(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsGoal.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_lisp(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/darknet_ros_msgs_eb
)

### Generating Services

### Generating Module File
_generate_module_lisp(darknet_ros_msgs_eb
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/darknet_ros_msgs_eb
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(darknet_ros_msgs_eb_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(darknet_ros_msgs_eb_generate_messages darknet_ros_msgs_eb_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionResult.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_lisp _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsAction.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_lisp _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionFeedback.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_lisp _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_lisp _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsResult.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_lisp _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_lisp _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsFeedback.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_lisp _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionGoal.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_lisp _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsGoal.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_lisp _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(darknet_ros_msgs_eb_genlisp)
add_dependencies(darknet_ros_msgs_eb_genlisp darknet_ros_msgs_eb_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS darknet_ros_msgs_eb_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsResult.msg;/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_nodejs(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_nodejs(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_nodejs(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsAction.msg"
  "${MSG_I_FLAGS}"
  "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionGoal.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsFeedback.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionFeedback.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsResult.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsGoal.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_nodejs(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsResult.msg"
  "${MSG_I_FLAGS}"
  "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg;/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_nodejs(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_nodejs(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_nodejs(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsGoal.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_nodejs(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/darknet_ros_msgs_eb
)

### Generating Services

### Generating Module File
_generate_module_nodejs(darknet_ros_msgs_eb
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/darknet_ros_msgs_eb
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(darknet_ros_msgs_eb_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(darknet_ros_msgs_eb_generate_messages darknet_ros_msgs_eb_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionResult.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_nodejs _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsAction.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_nodejs _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionFeedback.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_nodejs _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_nodejs _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsResult.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_nodejs _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_nodejs _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsFeedback.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_nodejs _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionGoal.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_nodejs _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsGoal.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_nodejs _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(darknet_ros_msgs_eb_gennodejs)
add_dependencies(darknet_ros_msgs_eb_gennodejs darknet_ros_msgs_eb_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS darknet_ros_msgs_eb_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsResult.msg;/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_py(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_py(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_py(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsAction.msg"
  "${MSG_I_FLAGS}"
  "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionGoal.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsFeedback.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionFeedback.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsResult.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsGoal.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_py(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsResult.msg"
  "${MSG_I_FLAGS}"
  "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg;/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_py(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_py(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_py(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsGoal.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/darknet_ros_msgs_eb
)
_generate_msg_py(darknet_ros_msgs_eb
  "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/darknet_ros_msgs_eb
)

### Generating Services

### Generating Module File
_generate_module_py(darknet_ros_msgs_eb
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/darknet_ros_msgs_eb
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(darknet_ros_msgs_eb_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(darknet_ros_msgs_eb_generate_messages darknet_ros_msgs_eb_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionResult.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_py _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsAction.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_py _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionFeedback.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_py _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_py _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsResult.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_py _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/src/yolo_eb/darknet_ros/darknet_ros_msgs/msg/BoundingBox.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_py _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsFeedback.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_py _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsActionGoal.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_py _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erlendb/Programmering/Master/test_ws/devel/share/darknet_ros_msgs_eb/msg/CheckForObjectsGoal.msg" NAME_WE)
add_dependencies(darknet_ros_msgs_eb_generate_messages_py _darknet_ros_msgs_eb_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(darknet_ros_msgs_eb_genpy)
add_dependencies(darknet_ros_msgs_eb_genpy darknet_ros_msgs_eb_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS darknet_ros_msgs_eb_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/darknet_ros_msgs_eb)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/darknet_ros_msgs_eb
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(darknet_ros_msgs_eb_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(darknet_ros_msgs_eb_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(darknet_ros_msgs_eb_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(darknet_ros_msgs_eb_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/darknet_ros_msgs_eb)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/darknet_ros_msgs_eb
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(darknet_ros_msgs_eb_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(darknet_ros_msgs_eb_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(darknet_ros_msgs_eb_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(darknet_ros_msgs_eb_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/darknet_ros_msgs_eb)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/darknet_ros_msgs_eb
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(darknet_ros_msgs_eb_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(darknet_ros_msgs_eb_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(darknet_ros_msgs_eb_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(darknet_ros_msgs_eb_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/darknet_ros_msgs_eb)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/darknet_ros_msgs_eb
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(darknet_ros_msgs_eb_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(darknet_ros_msgs_eb_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(darknet_ros_msgs_eb_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(darknet_ros_msgs_eb_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/darknet_ros_msgs_eb)
  install(CODE "execute_process(COMMAND \"/home/erlendb/anaconda2/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/darknet_ros_msgs_eb\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/darknet_ros_msgs_eb
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(darknet_ros_msgs_eb_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(darknet_ros_msgs_eb_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(darknet_ros_msgs_eb_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(darknet_ros_msgs_eb_generate_messages_py std_msgs_generate_messages_py)
endif()

; Auto-generated. Do not edit!


(cl:in-package pose_estimator-msg)


;//! \htmlinclude pipe_pose.msg.html

(cl:defclass <pipe_pose> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (pipe_x
    :reader pipe_x
    :initarg :pipe_x
    :type cl:float
    :initform 0.0)
   (pipe_y
    :reader pipe_y
    :initarg :pipe_y
    :type cl:float
    :initform 0.0)
   (pipe_z
    :reader pipe_z
    :initarg :pipe_z
    :type cl:float
    :initform 0.0)
   (pipe_theta_x
    :reader pipe_theta_x
    :initarg :pipe_theta_x
    :type cl:float
    :initform 0.0)
   (pipe_theta_y
    :reader pipe_theta_y
    :initarg :pipe_theta_y
    :type cl:float
    :initform 0.0)
   (pipe_theta_z
    :reader pipe_theta_z
    :initarg :pipe_theta_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass pipe_pose (<pipe_pose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pipe_pose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pipe_pose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pose_estimator-msg:<pipe_pose> is deprecated: use pose_estimator-msg:pipe_pose instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <pipe_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pose_estimator-msg:header-val is deprecated.  Use pose_estimator-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'pipe_x-val :lambda-list '(m))
(cl:defmethod pipe_x-val ((m <pipe_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pose_estimator-msg:pipe_x-val is deprecated.  Use pose_estimator-msg:pipe_x instead.")
  (pipe_x m))

(cl:ensure-generic-function 'pipe_y-val :lambda-list '(m))
(cl:defmethod pipe_y-val ((m <pipe_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pose_estimator-msg:pipe_y-val is deprecated.  Use pose_estimator-msg:pipe_y instead.")
  (pipe_y m))

(cl:ensure-generic-function 'pipe_z-val :lambda-list '(m))
(cl:defmethod pipe_z-val ((m <pipe_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pose_estimator-msg:pipe_z-val is deprecated.  Use pose_estimator-msg:pipe_z instead.")
  (pipe_z m))

(cl:ensure-generic-function 'pipe_theta_x-val :lambda-list '(m))
(cl:defmethod pipe_theta_x-val ((m <pipe_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pose_estimator-msg:pipe_theta_x-val is deprecated.  Use pose_estimator-msg:pipe_theta_x instead.")
  (pipe_theta_x m))

(cl:ensure-generic-function 'pipe_theta_y-val :lambda-list '(m))
(cl:defmethod pipe_theta_y-val ((m <pipe_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pose_estimator-msg:pipe_theta_y-val is deprecated.  Use pose_estimator-msg:pipe_theta_y instead.")
  (pipe_theta_y m))

(cl:ensure-generic-function 'pipe_theta_z-val :lambda-list '(m))
(cl:defmethod pipe_theta_z-val ((m <pipe_pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pose_estimator-msg:pipe_theta_z-val is deprecated.  Use pose_estimator-msg:pipe_theta_z instead.")
  (pipe_theta_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pipe_pose>) ostream)
  "Serializes a message object of type '<pipe_pose>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pipe_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pipe_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pipe_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pipe_theta_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pipe_theta_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pipe_theta_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pipe_pose>) istream)
  "Deserializes a message object of type '<pipe_pose>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pipe_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pipe_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pipe_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pipe_theta_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pipe_theta_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pipe_theta_z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pipe_pose>)))
  "Returns string type for a message object of type '<pipe_pose>"
  "pose_estimator/pipe_pose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pipe_pose)))
  "Returns string type for a message object of type 'pipe_pose"
  "pose_estimator/pipe_pose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pipe_pose>)))
  "Returns md5sum for a message object of type '<pipe_pose>"
  "51eaea2844bf3b987e1fb67e8d6ca8e2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pipe_pose)))
  "Returns md5sum for a message object of type 'pipe_pose"
  "51eaea2844bf3b987e1fb67e8d6ca8e2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pipe_pose>)))
  "Returns full string definition for message of type '<pipe_pose>"
  (cl:format cl:nil "Header header~%float32 pipe_x~%float32 pipe_y~%float32 pipe_z~%float32 pipe_theta_x~%float32 pipe_theta_y~%float32 pipe_theta_z~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pipe_pose)))
  "Returns full string definition for message of type 'pipe_pose"
  (cl:format cl:nil "Header header~%float32 pipe_x~%float32 pipe_y~%float32 pipe_z~%float32 pipe_theta_x~%float32 pipe_theta_y~%float32 pipe_theta_z~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pipe_pose>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pipe_pose>))
  "Converts a ROS message object to a list"
  (cl:list 'pipe_pose
    (cl:cons ':header (header msg))
    (cl:cons ':pipe_x (pipe_x msg))
    (cl:cons ':pipe_y (pipe_y msg))
    (cl:cons ':pipe_z (pipe_z msg))
    (cl:cons ':pipe_theta_x (pipe_theta_x msg))
    (cl:cons ':pipe_theta_y (pipe_theta_y msg))
    (cl:cons ':pipe_theta_z (pipe_theta_z msg))
))

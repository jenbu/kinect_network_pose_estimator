;; Auto-generated. Do not edit!


(when (boundp 'pose_estimator::pipe_pose)
  (if (not (find-package "POSE_ESTIMATOR"))
    (make-package "POSE_ESTIMATOR"))
  (shadow 'pipe_pose (find-package "POSE_ESTIMATOR")))
(unless (find-package "POSE_ESTIMATOR::PIPE_POSE")
  (make-package "POSE_ESTIMATOR::PIPE_POSE"))

(in-package "ROS")
;;//! \htmlinclude pipe_pose.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pose_estimator::pipe_pose
  :super ros::object
  :slots (_header _pipe_x _pipe_y _pipe_z _pipe_theta_x _pipe_theta_y _pipe_theta_z ))

(defmethod pose_estimator::pipe_pose
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:pipe_x __pipe_x) 0.0)
    ((:pipe_y __pipe_y) 0.0)
    ((:pipe_z __pipe_z) 0.0)
    ((:pipe_theta_x __pipe_theta_x) 0.0)
    ((:pipe_theta_y __pipe_theta_y) 0.0)
    ((:pipe_theta_z __pipe_theta_z) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _pipe_x (float __pipe_x))
   (setq _pipe_y (float __pipe_y))
   (setq _pipe_z (float __pipe_z))
   (setq _pipe_theta_x (float __pipe_theta_x))
   (setq _pipe_theta_y (float __pipe_theta_y))
   (setq _pipe_theta_z (float __pipe_theta_z))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:pipe_x
   (&optional __pipe_x)
   (if __pipe_x (setq _pipe_x __pipe_x)) _pipe_x)
  (:pipe_y
   (&optional __pipe_y)
   (if __pipe_y (setq _pipe_y __pipe_y)) _pipe_y)
  (:pipe_z
   (&optional __pipe_z)
   (if __pipe_z (setq _pipe_z __pipe_z)) _pipe_z)
  (:pipe_theta_x
   (&optional __pipe_theta_x)
   (if __pipe_theta_x (setq _pipe_theta_x __pipe_theta_x)) _pipe_theta_x)
  (:pipe_theta_y
   (&optional __pipe_theta_y)
   (if __pipe_theta_y (setq _pipe_theta_y __pipe_theta_y)) _pipe_theta_y)
  (:pipe_theta_z
   (&optional __pipe_theta_z)
   (if __pipe_theta_z (setq _pipe_theta_z __pipe_theta_z)) _pipe_theta_z)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _pipe_x
    4
    ;; float32 _pipe_y
    4
    ;; float32 _pipe_z
    4
    ;; float32 _pipe_theta_x
    4
    ;; float32 _pipe_theta_y
    4
    ;; float32 _pipe_theta_z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _pipe_x
       (sys::poke _pipe_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pipe_y
       (sys::poke _pipe_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pipe_z
       (sys::poke _pipe_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pipe_theta_x
       (sys::poke _pipe_theta_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pipe_theta_y
       (sys::poke _pipe_theta_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pipe_theta_z
       (sys::poke _pipe_theta_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _pipe_x
     (setq _pipe_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pipe_y
     (setq _pipe_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pipe_z
     (setq _pipe_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pipe_theta_x
     (setq _pipe_theta_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pipe_theta_y
     (setq _pipe_theta_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pipe_theta_z
     (setq _pipe_theta_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get pose_estimator::pipe_pose :md5sum-) "51eaea2844bf3b987e1fb67e8d6ca8e2")
(setf (get pose_estimator::pipe_pose :datatype-) "pose_estimator/pipe_pose")
(setf (get pose_estimator::pipe_pose :definition-)
      "Header header
float32 pipe_x
float32 pipe_y
float32 pipe_z
float32 pipe_theta_x
float32 pipe_theta_y
float32 pipe_theta_z
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :pose_estimator/pipe_pose "51eaea2844bf3b987e1fb67e8d6ca8e2")



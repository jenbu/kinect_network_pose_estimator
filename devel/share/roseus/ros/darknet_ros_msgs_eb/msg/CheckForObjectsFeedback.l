;; Auto-generated. Do not edit!


(when (boundp 'darknet_ros_msgs_eb::CheckForObjectsFeedback)
  (if (not (find-package "DARKNET_ROS_MSGS_EB"))
    (make-package "DARKNET_ROS_MSGS_EB"))
  (shadow 'CheckForObjectsFeedback (find-package "DARKNET_ROS_MSGS_EB")))
(unless (find-package "DARKNET_ROS_MSGS_EB::CHECKFOROBJECTSFEEDBACK")
  (make-package "DARKNET_ROS_MSGS_EB::CHECKFOROBJECTSFEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude CheckForObjectsFeedback.msg.html


(defclass darknet_ros_msgs_eb::CheckForObjectsFeedback
  :super ros::object
  :slots ())

(defmethod darknet_ros_msgs_eb::CheckForObjectsFeedback
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get darknet_ros_msgs_eb::CheckForObjectsFeedback :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get darknet_ros_msgs_eb::CheckForObjectsFeedback :datatype-) "darknet_ros_msgs_eb/CheckForObjectsFeedback")
(setf (get darknet_ros_msgs_eb::CheckForObjectsFeedback :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
# Feedback definition


")



(provide :darknet_ros_msgs_eb/CheckForObjectsFeedback "d41d8cd98f00b204e9800998ecf8427e")


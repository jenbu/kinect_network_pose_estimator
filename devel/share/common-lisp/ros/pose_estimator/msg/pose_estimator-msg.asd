
(cl:in-package :asdf)

(defsystem "pose_estimator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "pipe_pose" :depends-on ("_package_pipe_pose"))
    (:file "_package_pipe_pose" :depends-on ("_package"))
  ))
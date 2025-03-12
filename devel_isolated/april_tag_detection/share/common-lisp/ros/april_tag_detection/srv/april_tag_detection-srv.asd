
(cl:in-package :asdf)

(defsystem "april_tag_detection-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "GetAprilTag" :depends-on ("_package_GetAprilTag"))
    (:file "_package_GetAprilTag" :depends-on ("_package"))
  ))
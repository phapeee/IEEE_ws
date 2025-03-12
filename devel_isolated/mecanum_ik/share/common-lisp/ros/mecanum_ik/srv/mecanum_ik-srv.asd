
(cl:in-package :asdf)

(defsystem "mecanum_ik-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "EmptySrv" :depends-on ("_package_EmptySrv"))
    (:file "_package_EmptySrv" :depends-on ("_package"))
  ))
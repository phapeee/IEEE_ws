
(cl:in-package :asdf)

(defsystem "mecanum_ik-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "vector4_msg" :depends-on ("_package_vector4_msg"))
    (:file "_package_vector4_msg" :depends-on ("_package"))
  ))
;; Auto-generated. Do not edit!


(when (boundp 'mecanum_ik::EmptySrv)
  (if (not (find-package "MECANUM_IK"))
    (make-package "MECANUM_IK"))
  (shadow 'EmptySrv (find-package "MECANUM_IK")))
(unless (find-package "MECANUM_IK::EMPTYSRV")
  (make-package "MECANUM_IK::EMPTYSRV"))
(unless (find-package "MECANUM_IK::EMPTYSRVREQUEST")
  (make-package "MECANUM_IK::EMPTYSRVREQUEST"))
(unless (find-package "MECANUM_IK::EMPTYSRVRESPONSE")
  (make-package "MECANUM_IK::EMPTYSRVRESPONSE"))

(in-package "ROS")





(defclass mecanum_ik::EmptySrvRequest
  :super ros::object
  :slots ())

(defmethod mecanum_ik::EmptySrvRequest
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

(defclass mecanum_ik::EmptySrvResponse
  :super ros::object
  :slots ())

(defmethod mecanum_ik::EmptySrvResponse
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

(defclass mecanum_ik::EmptySrv
  :super ros::object
  :slots ())

(setf (get mecanum_ik::EmptySrv :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get mecanum_ik::EmptySrv :datatype-) "mecanum_ik/EmptySrv")
(setf (get mecanum_ik::EmptySrv :request) mecanum_ik::EmptySrvRequest)
(setf (get mecanum_ik::EmptySrv :response) mecanum_ik::EmptySrvResponse)

(defmethod mecanum_ik::EmptySrvRequest
  (:response () (instance mecanum_ik::EmptySrvResponse :init)))

(setf (get mecanum_ik::EmptySrvRequest :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get mecanum_ik::EmptySrvRequest :datatype-) "mecanum_ik/EmptySrvRequest")
(setf (get mecanum_ik::EmptySrvRequest :definition-)
      "# Empty service, no request and response
---

")

(setf (get mecanum_ik::EmptySrvResponse :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get mecanum_ik::EmptySrvResponse :datatype-) "mecanum_ik/EmptySrvResponse")
(setf (get mecanum_ik::EmptySrvResponse :definition-)
      "# Empty service, no request and response
---

")



(provide :mecanum_ik/EmptySrv "d41d8cd98f00b204e9800998ecf8427e")



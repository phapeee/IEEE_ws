;; Auto-generated. Do not edit!


(when (boundp 'april_tag_detection::GetAprilTag)
  (if (not (find-package "APRIL_TAG_DETECTION"))
    (make-package "APRIL_TAG_DETECTION"))
  (shadow 'GetAprilTag (find-package "APRIL_TAG_DETECTION")))
(unless (find-package "APRIL_TAG_DETECTION::GETAPRILTAG")
  (make-package "APRIL_TAG_DETECTION::GETAPRILTAG"))
(unless (find-package "APRIL_TAG_DETECTION::GETAPRILTAGREQUEST")
  (make-package "APRIL_TAG_DETECTION::GETAPRILTAGREQUEST"))
(unless (find-package "APRIL_TAG_DETECTION::GETAPRILTAGRESPONSE")
  (make-package "APRIL_TAG_DETECTION::GETAPRILTAGRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass april_tag_detection::GetAprilTagRequest
  :super ros::object
  :slots (_tag_id _cam_id ))

(defmethod april_tag_detection::GetAprilTagRequest
  (:init
   (&key
    ((:tag_id __tag_id) "")
    ((:cam_id __cam_id) "")
    )
   (send-super :init)
   (setq _tag_id (string __tag_id))
   (setq _cam_id (string __cam_id))
   self)
  (:tag_id
   (&optional __tag_id)
   (if __tag_id (setq _tag_id __tag_id)) _tag_id)
  (:cam_id
   (&optional __cam_id)
   (if __cam_id (setq _cam_id __cam_id)) _cam_id)
  (:serialization-length
   ()
   (+
    ;; string _tag_id
    4 (length _tag_id)
    ;; string _cam_id
    4 (length _cam_id)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _tag_id
       (write-long (length _tag_id) s) (princ _tag_id s)
     ;; string _cam_id
       (write-long (length _cam_id) s) (princ _cam_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _tag_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _tag_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _cam_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _cam_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass april_tag_detection::GetAprilTagResponse
  :super ros::object
  :slots (_tag_count _tag_id _cam_id _poses ))

(defmethod april_tag_detection::GetAprilTagResponse
  (:init
   (&key
    ((:tag_count __tag_count) 0)
    ((:tag_id __tag_id) (make-array 0 :initial-element 0 :element-type :char))
    ((:cam_id __cam_id) (make-array 0 :initial-element 0 :element-type :char))
    ((:poses __poses) ())
    )
   (send-super :init)
   (setq _tag_count (round __tag_count))
   (setq _tag_id __tag_id)
   (setq _cam_id __cam_id)
   (setq _poses __poses)
   self)
  (:tag_count
   (&optional __tag_count)
   (if __tag_count (setq _tag_count __tag_count)) _tag_count)
  (:tag_id
   (&optional __tag_id)
   (if __tag_id (setq _tag_id __tag_id)) _tag_id)
  (:cam_id
   (&optional __cam_id)
   (if __cam_id (setq _cam_id __cam_id)) _cam_id)
  (:poses
   (&rest __poses)
   (if (keywordp (car __poses))
       (send* _poses __poses)
     (progn
       (if __poses (setq _poses (car __poses)))
       _poses)))
  (:serialization-length
   ()
   (+
    ;; uint8 _tag_count
    1
    ;; uint8[] _tag_id
    (* 1    (length _tag_id)) 4
    ;; uint8[] _cam_id
    (* 1    (length _cam_id)) 4
    ;; geometry_msgs/Pose[] _poses
    (apply #'+ (send-all _poses :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _tag_count
       (write-byte _tag_count s)
     ;; uint8[] _tag_id
     (write-long (length _tag_id) s)
     (princ _tag_id s)
     ;; uint8[] _cam_id
     (write-long (length _cam_id) s)
     (princ _cam_id s)
     ;; geometry_msgs/Pose[] _poses
     (write-long (length _poses) s)
     (dolist (elem _poses)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _tag_count
     (setq _tag_count (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8[] _tag_id
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _tag_id (make-array n :element-type :char))
     (replace _tag_id buf :start2 ptr-) (incf ptr- n))
   ;; uint8[] _cam_id
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _cam_id (make-array n :element-type :char))
     (replace _cam_id buf :start2 ptr-) (incf ptr- n))
   ;; geometry_msgs/Pose[] _poses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _poses (let (r) (dotimes (i n) (push (instance geometry_msgs::Pose :init) r)) r))
     (dolist (elem- _poses)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass april_tag_detection::GetAprilTag
  :super ros::object
  :slots ())

(setf (get april_tag_detection::GetAprilTag :md5sum-) "11adae4fdc00ea7e90b1c993476c8886")
(setf (get april_tag_detection::GetAprilTag :datatype-) "april_tag_detection/GetAprilTag")
(setf (get april_tag_detection::GetAprilTag :request) april_tag_detection::GetAprilTagRequest)
(setf (get april_tag_detection::GetAprilTag :response) april_tag_detection::GetAprilTagResponse)

(defmethod april_tag_detection::GetAprilTagRequest
  (:response () (instance april_tag_detection::GetAprilTagResponse :init)))

(setf (get april_tag_detection::GetAprilTagRequest :md5sum-) "11adae4fdc00ea7e90b1c993476c8886")
(setf (get april_tag_detection::GetAprilTagRequest :datatype-) "april_tag_detection/GetAprilTagRequest")
(setf (get april_tag_detection::GetAprilTagRequest :definition-)
      "string tag_id
string cam_id
---
uint8 tag_count
uint8[] tag_id
uint8[] cam_id
geometry_msgs/Pose[] poses


================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get april_tag_detection::GetAprilTagResponse :md5sum-) "11adae4fdc00ea7e90b1c993476c8886")
(setf (get april_tag_detection::GetAprilTagResponse :datatype-) "april_tag_detection/GetAprilTagResponse")
(setf (get april_tag_detection::GetAprilTagResponse :definition-)
      "string tag_id
string cam_id
---
uint8 tag_count
uint8[] tag_id
uint8[] cam_id
geometry_msgs/Pose[] poses


================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :april_tag_detection/GetAprilTag "11adae4fdc00ea7e90b1c993476c8886")



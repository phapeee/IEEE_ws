; Auto-generated. Do not edit!


(cl:in-package april_tag_detection-srv)


;//! \htmlinclude GetAprilTag-request.msg.html

(cl:defclass <GetAprilTag-request> (roslisp-msg-protocol:ros-message)
  ((tag_id
    :reader tag_id
    :initarg :tag_id
    :type cl:string
    :initform "")
   (cam_id
    :reader cam_id
    :initarg :cam_id
    :type cl:string
    :initform ""))
)

(cl:defclass GetAprilTag-request (<GetAprilTag-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAprilTag-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAprilTag-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name april_tag_detection-srv:<GetAprilTag-request> is deprecated: use april_tag_detection-srv:GetAprilTag-request instead.")))

(cl:ensure-generic-function 'tag_id-val :lambda-list '(m))
(cl:defmethod tag_id-val ((m <GetAprilTag-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader april_tag_detection-srv:tag_id-val is deprecated.  Use april_tag_detection-srv:tag_id instead.")
  (tag_id m))

(cl:ensure-generic-function 'cam_id-val :lambda-list '(m))
(cl:defmethod cam_id-val ((m <GetAprilTag-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader april_tag_detection-srv:cam_id-val is deprecated.  Use april_tag_detection-srv:cam_id instead.")
  (cam_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAprilTag-request>) ostream)
  "Serializes a message object of type '<GetAprilTag-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'tag_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'tag_id))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cam_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cam_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAprilTag-request>) istream)
  "Deserializes a message object of type '<GetAprilTag-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tag_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'tag_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cam_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cam_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAprilTag-request>)))
  "Returns string type for a service object of type '<GetAprilTag-request>"
  "april_tag_detection/GetAprilTagRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAprilTag-request)))
  "Returns string type for a service object of type 'GetAprilTag-request"
  "april_tag_detection/GetAprilTagRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAprilTag-request>)))
  "Returns md5sum for a message object of type '<GetAprilTag-request>"
  "8da4d9ec9691782775e0aa5067c83359")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAprilTag-request)))
  "Returns md5sum for a message object of type 'GetAprilTag-request"
  "8da4d9ec9691782775e0aa5067c83359")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAprilTag-request>)))
  "Returns full string definition for message of type '<GetAprilTag-request>"
  (cl:format cl:nil "string tag_id~%string cam_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAprilTag-request)))
  "Returns full string definition for message of type 'GetAprilTag-request"
  (cl:format cl:nil "string tag_id~%string cam_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAprilTag-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'tag_id))
     4 (cl:length (cl:slot-value msg 'cam_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAprilTag-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAprilTag-request
    (cl:cons ':tag_id (tag_id msg))
    (cl:cons ':cam_id (cam_id msg))
))
;//! \htmlinclude GetAprilTag-response.msg.html

(cl:defclass <GetAprilTag-response> (roslisp-msg-protocol:ros-message)
  ((tag_id
    :reader tag_id
    :initarg :tag_id
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (cam_id
    :reader cam_id
    :initarg :cam_id
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (poses
    :reader poses
    :initarg :poses
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose))))
)

(cl:defclass GetAprilTag-response (<GetAprilTag-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAprilTag-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAprilTag-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name april_tag_detection-srv:<GetAprilTag-response> is deprecated: use april_tag_detection-srv:GetAprilTag-response instead.")))

(cl:ensure-generic-function 'tag_id-val :lambda-list '(m))
(cl:defmethod tag_id-val ((m <GetAprilTag-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader april_tag_detection-srv:tag_id-val is deprecated.  Use april_tag_detection-srv:tag_id instead.")
  (tag_id m))

(cl:ensure-generic-function 'cam_id-val :lambda-list '(m))
(cl:defmethod cam_id-val ((m <GetAprilTag-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader april_tag_detection-srv:cam_id-val is deprecated.  Use april_tag_detection-srv:cam_id instead.")
  (cam_id m))

(cl:ensure-generic-function 'poses-val :lambda-list '(m))
(cl:defmethod poses-val ((m <GetAprilTag-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader april_tag_detection-srv:poses-val is deprecated.  Use april_tag_detection-srv:poses instead.")
  (poses m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAprilTag-response>) ostream)
  "Serializes a message object of type '<GetAprilTag-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tag_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'tag_id))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cam_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'cam_id))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'poses))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAprilTag-response>) istream)
  "Deserializes a message object of type '<GetAprilTag-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tag_id) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tag_id)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cam_id) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cam_id)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'poses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'poses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAprilTag-response>)))
  "Returns string type for a service object of type '<GetAprilTag-response>"
  "april_tag_detection/GetAprilTagResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAprilTag-response)))
  "Returns string type for a service object of type 'GetAprilTag-response"
  "april_tag_detection/GetAprilTagResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAprilTag-response>)))
  "Returns md5sum for a message object of type '<GetAprilTag-response>"
  "8da4d9ec9691782775e0aa5067c83359")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAprilTag-response)))
  "Returns md5sum for a message object of type 'GetAprilTag-response"
  "8da4d9ec9691782775e0aa5067c83359")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAprilTag-response>)))
  "Returns full string definition for message of type '<GetAprilTag-response>"
  (cl:format cl:nil "uint8[] tag_id~%uint8[] cam_id~%geometry_msgs/Pose[] poses~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAprilTag-response)))
  "Returns full string definition for message of type 'GetAprilTag-response"
  (cl:format cl:nil "uint8[] tag_id~%uint8[] cam_id~%geometry_msgs/Pose[] poses~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAprilTag-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tag_id) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cam_id) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAprilTag-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAprilTag-response
    (cl:cons ':tag_id (tag_id msg))
    (cl:cons ':cam_id (cam_id msg))
    (cl:cons ':poses (poses msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetAprilTag)))
  'GetAprilTag-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetAprilTag)))
  'GetAprilTag-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAprilTag)))
  "Returns string type for a service object of type '<GetAprilTag>"
  "april_tag_detection/GetAprilTag")
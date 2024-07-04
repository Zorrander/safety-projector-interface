; Auto-generated. Do not edit!


(cl:in-package tuni_whitegoods_msgs-srv)


;//! \htmlinclude PointsDepthMap-request.msg.html

(cl:defclass <PointsDepthMap-request> (roslisp-msg-protocol:ros-message)
  ((poi_rgb
    :reader poi_rgb
    :initarg :poi_rgb
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass PointsDepthMap-request (<PointsDepthMap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PointsDepthMap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PointsDepthMap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tuni_whitegoods_msgs-srv:<PointsDepthMap-request> is deprecated: use tuni_whitegoods_msgs-srv:PointsDepthMap-request instead.")))

(cl:ensure-generic-function 'poi_rgb-val :lambda-list '(m))
(cl:defmethod poi_rgb-val ((m <PointsDepthMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tuni_whitegoods_msgs-srv:poi_rgb-val is deprecated.  Use tuni_whitegoods_msgs-srv:poi_rgb instead.")
  (poi_rgb m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PointsDepthMap-request>) ostream)
  "Serializes a message object of type '<PointsDepthMap-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'poi_rgb))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'poi_rgb))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PointsDepthMap-request>) istream)
  "Deserializes a message object of type '<PointsDepthMap-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'poi_rgb) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'poi_rgb)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PointsDepthMap-request>)))
  "Returns string type for a service object of type '<PointsDepthMap-request>"
  "tuni_whitegoods_msgs/PointsDepthMapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PointsDepthMap-request)))
  "Returns string type for a service object of type 'PointsDepthMap-request"
  "tuni_whitegoods_msgs/PointsDepthMapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PointsDepthMap-request>)))
  "Returns md5sum for a message object of type '<PointsDepthMap-request>"
  "053187b90257a916cb98c3355d430439")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PointsDepthMap-request)))
  "Returns md5sum for a message object of type 'PointsDepthMap-request"
  "053187b90257a916cb98c3355d430439")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PointsDepthMap-request>)))
  "Returns full string definition for message of type '<PointsDepthMap-request>"
  (cl:format cl:nil "geometry_msgs/Point[] poi_rgb~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PointsDepthMap-request)))
  "Returns full string definition for message of type 'PointsDepthMap-request"
  (cl:format cl:nil "geometry_msgs/Point[] poi_rgb~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PointsDepthMap-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'poi_rgb) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PointsDepthMap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PointsDepthMap-request
    (cl:cons ':poi_rgb (poi_rgb msg))
))
;//! \htmlinclude PointsDepthMap-response.msg.html

(cl:defclass <PointsDepthMap-response> (roslisp-msg-protocol:ros-message)
  ((poi_depth_map
    :reader poi_depth_map
    :initarg :poi_depth_map
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass PointsDepthMap-response (<PointsDepthMap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PointsDepthMap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PointsDepthMap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tuni_whitegoods_msgs-srv:<PointsDepthMap-response> is deprecated: use tuni_whitegoods_msgs-srv:PointsDepthMap-response instead.")))

(cl:ensure-generic-function 'poi_depth_map-val :lambda-list '(m))
(cl:defmethod poi_depth_map-val ((m <PointsDepthMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tuni_whitegoods_msgs-srv:poi_depth_map-val is deprecated.  Use tuni_whitegoods_msgs-srv:poi_depth_map instead.")
  (poi_depth_map m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PointsDepthMap-response>) ostream)
  "Serializes a message object of type '<PointsDepthMap-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'poi_depth_map))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'poi_depth_map))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PointsDepthMap-response>) istream)
  "Deserializes a message object of type '<PointsDepthMap-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'poi_depth_map) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'poi_depth_map)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PointsDepthMap-response>)))
  "Returns string type for a service object of type '<PointsDepthMap-response>"
  "tuni_whitegoods_msgs/PointsDepthMapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PointsDepthMap-response)))
  "Returns string type for a service object of type 'PointsDepthMap-response"
  "tuni_whitegoods_msgs/PointsDepthMapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PointsDepthMap-response>)))
  "Returns md5sum for a message object of type '<PointsDepthMap-response>"
  "053187b90257a916cb98c3355d430439")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PointsDepthMap-response)))
  "Returns md5sum for a message object of type 'PointsDepthMap-response"
  "053187b90257a916cb98c3355d430439")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PointsDepthMap-response>)))
  "Returns full string definition for message of type '<PointsDepthMap-response>"
  (cl:format cl:nil "geometry_msgs/Point[] poi_depth_map~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PointsDepthMap-response)))
  "Returns full string definition for message of type 'PointsDepthMap-response"
  (cl:format cl:nil "geometry_msgs/Point[] poi_depth_map~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PointsDepthMap-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'poi_depth_map) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PointsDepthMap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PointsDepthMap-response
    (cl:cons ':poi_depth_map (poi_depth_map msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PointsDepthMap)))
  'PointsDepthMap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PointsDepthMap)))
  'PointsDepthMap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PointsDepthMap)))
  "Returns string type for a service object of type '<PointsDepthMap>"
  "tuni_whitegoods_msgs/PointsDepthMap")
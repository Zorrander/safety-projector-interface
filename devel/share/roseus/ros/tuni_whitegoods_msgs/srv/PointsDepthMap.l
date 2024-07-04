;; Auto-generated. Do not edit!


(when (boundp 'tuni_whitegoods_msgs::PointsDepthMap)
  (if (not (find-package "TUNI_WHITEGOODS_MSGS"))
    (make-package "TUNI_WHITEGOODS_MSGS"))
  (shadow 'PointsDepthMap (find-package "TUNI_WHITEGOODS_MSGS")))
(unless (find-package "TUNI_WHITEGOODS_MSGS::POINTSDEPTHMAP")
  (make-package "TUNI_WHITEGOODS_MSGS::POINTSDEPTHMAP"))
(unless (find-package "TUNI_WHITEGOODS_MSGS::POINTSDEPTHMAPREQUEST")
  (make-package "TUNI_WHITEGOODS_MSGS::POINTSDEPTHMAPREQUEST"))
(unless (find-package "TUNI_WHITEGOODS_MSGS::POINTSDEPTHMAPRESPONSE")
  (make-package "TUNI_WHITEGOODS_MSGS::POINTSDEPTHMAPRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass tuni_whitegoods_msgs::PointsDepthMapRequest
  :super ros::object
  :slots (_poi_rgb ))

(defmethod tuni_whitegoods_msgs::PointsDepthMapRequest
  (:init
   (&key
    ((:poi_rgb __poi_rgb) ())
    )
   (send-super :init)
   (setq _poi_rgb __poi_rgb)
   self)
  (:poi_rgb
   (&rest __poi_rgb)
   (if (keywordp (car __poi_rgb))
       (send* _poi_rgb __poi_rgb)
     (progn
       (if __poi_rgb (setq _poi_rgb (car __poi_rgb)))
       _poi_rgb)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point[] _poi_rgb
    (apply #'+ (send-all _poi_rgb :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point[] _poi_rgb
     (write-long (length _poi_rgb) s)
     (dolist (elem _poi_rgb)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point[] _poi_rgb
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _poi_rgb (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _poi_rgb)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass tuni_whitegoods_msgs::PointsDepthMapResponse
  :super ros::object
  :slots (_poi_depth_map ))

(defmethod tuni_whitegoods_msgs::PointsDepthMapResponse
  (:init
   (&key
    ((:poi_depth_map __poi_depth_map) ())
    )
   (send-super :init)
   (setq _poi_depth_map __poi_depth_map)
   self)
  (:poi_depth_map
   (&rest __poi_depth_map)
   (if (keywordp (car __poi_depth_map))
       (send* _poi_depth_map __poi_depth_map)
     (progn
       (if __poi_depth_map (setq _poi_depth_map (car __poi_depth_map)))
       _poi_depth_map)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point[] _poi_depth_map
    (apply #'+ (send-all _poi_depth_map :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point[] _poi_depth_map
     (write-long (length _poi_depth_map) s)
     (dolist (elem _poi_depth_map)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point[] _poi_depth_map
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _poi_depth_map (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _poi_depth_map)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass tuni_whitegoods_msgs::PointsDepthMap
  :super ros::object
  :slots ())

(setf (get tuni_whitegoods_msgs::PointsDepthMap :md5sum-) "053187b90257a916cb98c3355d430439")
(setf (get tuni_whitegoods_msgs::PointsDepthMap :datatype-) "tuni_whitegoods_msgs/PointsDepthMap")
(setf (get tuni_whitegoods_msgs::PointsDepthMap :request) tuni_whitegoods_msgs::PointsDepthMapRequest)
(setf (get tuni_whitegoods_msgs::PointsDepthMap :response) tuni_whitegoods_msgs::PointsDepthMapResponse)

(defmethod tuni_whitegoods_msgs::PointsDepthMapRequest
  (:response () (instance tuni_whitegoods_msgs::PointsDepthMapResponse :init)))

(setf (get tuni_whitegoods_msgs::PointsDepthMapRequest :md5sum-) "053187b90257a916cb98c3355d430439")
(setf (get tuni_whitegoods_msgs::PointsDepthMapRequest :datatype-) "tuni_whitegoods_msgs/PointsDepthMapRequest")
(setf (get tuni_whitegoods_msgs::PointsDepthMapRequest :definition-)
      "geometry_msgs/Point[] poi_rgb

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
geometry_msgs/Point[] poi_depth_map

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")

(setf (get tuni_whitegoods_msgs::PointsDepthMapResponse :md5sum-) "053187b90257a916cb98c3355d430439")
(setf (get tuni_whitegoods_msgs::PointsDepthMapResponse :datatype-) "tuni_whitegoods_msgs/PointsDepthMapResponse")
(setf (get tuni_whitegoods_msgs::PointsDepthMapResponse :definition-)
      "geometry_msgs/Point[] poi_rgb

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
geometry_msgs/Point[] poi_depth_map

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
")



(provide :tuni_whitegoods_msgs/PointsDepthMap "053187b90257a916cb98c3355d430439")



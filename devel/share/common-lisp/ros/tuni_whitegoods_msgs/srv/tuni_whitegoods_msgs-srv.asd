
(cl:in-package :asdf)

(defsystem "tuni_whitegoods_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "PointsDepthMap" :depends-on ("_package_PointsDepthMap"))
    (:file "_package_PointsDepthMap" :depends-on ("_package"))
  ))
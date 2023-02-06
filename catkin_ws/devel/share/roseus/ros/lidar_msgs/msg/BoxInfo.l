;; Auto-generated. Do not edit!


(when (boundp 'lidar_msgs::BoxInfo)
  (if (not (find-package "LIDAR_MSGS"))
    (make-package "LIDAR_MSGS"))
  (shadow 'BoxInfo (find-package "LIDAR_MSGS")))
(unless (find-package "LIDAR_MSGS::BOXINFO")
  (make-package "LIDAR_MSGS::BOXINFO"))

(in-package "ROS")
;;//! \htmlinclude BoxInfo.msg.html


(defclass lidar_msgs::BoxInfo
  :super ros::object
  :slots (_box ))

(defmethod lidar_msgs::BoxInfo
  (:init
   (&key
    ((:box __box) (instance lidar_msgs::box :init))
    )
   (send-super :init)
   (setq _box __box)
   self)
  (:box
   (&rest __box)
   (if (keywordp (car __box))
       (send* _box __box)
     (progn
       (if __box (setq _box (car __box)))
       _box)))
  (:serialization-length
   ()
   (+
    ;; lidar_msgs/box _box
    (send _box :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; lidar_msgs/box _box
       (send _box :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; lidar_msgs/box _box
     (send _box :deserialize buf ptr-) (incf ptr- (send _box :serialization-length))
   ;;
   self)
  )

(setf (get lidar_msgs::BoxInfo :md5sum-) "7f621b8f09e614f07fb08a2b7f9a6583")
(setf (get lidar_msgs::BoxInfo :datatype-) "lidar_msgs/BoxInfo")
(setf (get lidar_msgs::BoxInfo :definition-)
      "box box

================================================================================
MSG: lidar_msgs/box
float32 x_min
float32 y_min
float32 z_min
float32 x_max
float32 y_max
float32 z_max

")



(provide :lidar_msgs/BoxInfo "7f621b8f09e614f07fb08a2b7f9a6583")



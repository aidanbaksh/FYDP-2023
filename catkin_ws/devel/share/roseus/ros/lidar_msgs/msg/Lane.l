;; Auto-generated. Do not edit!


(when (boundp 'lidar_msgs::Lane)
  (if (not (find-package "LIDAR_MSGS"))
    (make-package "LIDAR_MSGS"))
  (shadow 'Lane (find-package "LIDAR_MSGS")))
(unless (find-package "LIDAR_MSGS::LANE")
  (make-package "LIDAR_MSGS::LANE"))

(in-package "ROS")
;;//! \htmlinclude Lane.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass lidar_msgs::Lane
  :super ros::object
  :slots (_header _points ))

(defmethod lidar_msgs::Lane
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:points __points) (let (r) (dotimes (i 200) (push (instance lidar_msgs::PointStamped :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _points __points)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; lidar_msgs/PointStamped[200] _points
    (apply #'+ (send-all _points :serialization-length))
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; lidar_msgs/PointStamped[200] _points
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; lidar_msgs/PointStamped[200] _points
   (dotimes (i 200)
     (send (elt _points i) :deserialize buf ptr-) (incf ptr- (send (elt _points i) :serialization-length))
     )
   ;;
   self)
  )

(setf (get lidar_msgs::Lane :md5sum-) "75af1c1affc9fe6f9de6b476e4c14f22")
(setf (get lidar_msgs::Lane :datatype-) "lidar_msgs/Lane")
(setf (get lidar_msgs::Lane :definition-)
      "std_msgs/Header header
PointStamped[200] points
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: lidar_msgs/PointStamped
float64 x
float64 y
float64 z
float64 intensity
")



(provide :lidar_msgs/Lane "75af1c1affc9fe6f9de6b476e4c14f22")



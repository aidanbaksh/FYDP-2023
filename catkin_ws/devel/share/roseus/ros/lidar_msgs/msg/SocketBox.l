;; Auto-generated. Do not edit!


(when (boundp 'lidar_msgs::SocketBox)
  (if (not (find-package "LIDAR_MSGS"))
    (make-package "LIDAR_MSGS"))
  (shadow 'SocketBox (find-package "LIDAR_MSGS")))
(unless (find-package "LIDAR_MSGS::SOCKETBOX")
  (make-package "LIDAR_MSGS::SOCKETBOX"))

(in-package "ROS")
;;//! \htmlinclude SocketBox.msg.html


(defclass lidar_msgs::SocketBox
  :super ros::object
  :slots (_num_of_box _box_info ))

(defmethod lidar_msgs::SocketBox
  (:init
   (&key
    ((:num_of_box __num_of_box) 0)
    ((:box_info __box_info) (let (r) (dotimes (i 0) (push (instance lidar_msgs::BoxInfo :init) r)) r))
    )
   (send-super :init)
   (setq _num_of_box (round __num_of_box))
   (setq _box_info __box_info)
   self)
  (:num_of_box
   (&optional __num_of_box)
   (if __num_of_box (setq _num_of_box __num_of_box)) _num_of_box)
  (:box_info
   (&rest __box_info)
   (if (keywordp (car __box_info))
       (send* _box_info __box_info)
     (progn
       (if __box_info (setq _box_info (car __box_info)))
       _box_info)))
  (:serialization-length
   ()
   (+
    ;; int32 _num_of_box
    4
    ;; lidar_msgs/BoxInfo[] _box_info
    (apply #'+ (send-all _box_info :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _num_of_box
       (write-long _num_of_box s)
     ;; lidar_msgs/BoxInfo[] _box_info
     (write-long (length _box_info) s)
     (dolist (elem _box_info)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _num_of_box
     (setq _num_of_box (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; lidar_msgs/BoxInfo[] _box_info
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _box_info (let (r) (dotimes (i n) (push (instance lidar_msgs::BoxInfo :init) r)) r))
     (dolist (elem- _box_info)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get lidar_msgs::SocketBox :md5sum-) "7e7c1fed7bbc79bff735bfacc0aaebc3")
(setf (get lidar_msgs::SocketBox :datatype-) "lidar_msgs/SocketBox")
(setf (get lidar_msgs::SocketBox :definition-)
      "int32 num_of_box
BoxInfo[] box_info

================================================================================
MSG: lidar_msgs/BoxInfo
box box

================================================================================
MSG: lidar_msgs/box
float32 x_min
float32 y_min
float32 z_min
float32 x_max
float32 y_max
float32 z_max

")



(provide :lidar_msgs/SocketBox "7e7c1fed7bbc79bff735bfacc0aaebc3")



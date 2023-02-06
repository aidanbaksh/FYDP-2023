;; Auto-generated. Do not edit!


(when (boundp 'lidar_msgs::box)
  (if (not (find-package "LIDAR_MSGS"))
    (make-package "LIDAR_MSGS"))
  (shadow 'box (find-package "LIDAR_MSGS")))
(unless (find-package "LIDAR_MSGS::BOX")
  (make-package "LIDAR_MSGS::BOX"))

(in-package "ROS")
;;//! \htmlinclude box.msg.html


(defclass lidar_msgs::box
  :super ros::object
  :slots (_x_min _y_min _z_min _x_max _y_max _z_max ))

(defmethod lidar_msgs::box
  (:init
   (&key
    ((:x_min __x_min) 0.0)
    ((:y_min __y_min) 0.0)
    ((:z_min __z_min) 0.0)
    ((:x_max __x_max) 0.0)
    ((:y_max __y_max) 0.0)
    ((:z_max __z_max) 0.0)
    )
   (send-super :init)
   (setq _x_min (float __x_min))
   (setq _y_min (float __y_min))
   (setq _z_min (float __z_min))
   (setq _x_max (float __x_max))
   (setq _y_max (float __y_max))
   (setq _z_max (float __z_max))
   self)
  (:x_min
   (&optional __x_min)
   (if __x_min (setq _x_min __x_min)) _x_min)
  (:y_min
   (&optional __y_min)
   (if __y_min (setq _y_min __y_min)) _y_min)
  (:z_min
   (&optional __z_min)
   (if __z_min (setq _z_min __z_min)) _z_min)
  (:x_max
   (&optional __x_max)
   (if __x_max (setq _x_max __x_max)) _x_max)
  (:y_max
   (&optional __y_max)
   (if __y_max (setq _y_max __y_max)) _y_max)
  (:z_max
   (&optional __z_max)
   (if __z_max (setq _z_max __z_max)) _z_max)
  (:serialization-length
   ()
   (+
    ;; float32 _x_min
    4
    ;; float32 _y_min
    4
    ;; float32 _z_min
    4
    ;; float32 _x_max
    4
    ;; float32 _y_max
    4
    ;; float32 _z_max
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x_min
       (sys::poke _x_min (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_min
       (sys::poke _y_min (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z_min
       (sys::poke _z_min (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _x_max
       (sys::poke _x_max (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_max
       (sys::poke _y_max (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z_max
       (sys::poke _z_max (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x_min
     (setq _x_min (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_min
     (setq _y_min (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z_min
     (setq _z_min (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _x_max
     (setq _x_max (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_max
     (setq _y_max (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z_max
     (setq _z_max (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get lidar_msgs::box :md5sum-) "15f96afef8ea13b641fa2f30638908e0")
(setf (get lidar_msgs::box :datatype-) "lidar_msgs/box")
(setf (get lidar_msgs::box :definition-)
      "float32 x_min
float32 y_min
float32 z_min
float32 x_max
float32 y_max
float32 z_max

")



(provide :lidar_msgs/box "15f96afef8ea13b641fa2f30638908e0")



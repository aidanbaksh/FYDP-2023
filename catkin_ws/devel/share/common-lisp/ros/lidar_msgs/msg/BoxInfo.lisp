; Auto-generated. Do not edit!


(cl:in-package lidar_msgs-msg)


;//! \htmlinclude BoxInfo.msg.html

(cl:defclass <BoxInfo> (roslisp-msg-protocol:ros-message)
  ((box
    :reader box
    :initarg :box
    :type lidar_msgs-msg:box
    :initform (cl:make-instance 'lidar_msgs-msg:box)))
)

(cl:defclass BoxInfo (<BoxInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BoxInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BoxInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lidar_msgs-msg:<BoxInfo> is deprecated: use lidar_msgs-msg:BoxInfo instead.")))

(cl:ensure-generic-function 'box-val :lambda-list '(m))
(cl:defmethod box-val ((m <BoxInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_msgs-msg:box-val is deprecated.  Use lidar_msgs-msg:box instead.")
  (box m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BoxInfo>) ostream)
  "Serializes a message object of type '<BoxInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'box) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BoxInfo>) istream)
  "Deserializes a message object of type '<BoxInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'box) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BoxInfo>)))
  "Returns string type for a message object of type '<BoxInfo>"
  "lidar_msgs/BoxInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BoxInfo)))
  "Returns string type for a message object of type 'BoxInfo"
  "lidar_msgs/BoxInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BoxInfo>)))
  "Returns md5sum for a message object of type '<BoxInfo>"
  "7f621b8f09e614f07fb08a2b7f9a6583")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BoxInfo)))
  "Returns md5sum for a message object of type 'BoxInfo"
  "7f621b8f09e614f07fb08a2b7f9a6583")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BoxInfo>)))
  "Returns full string definition for message of type '<BoxInfo>"
  (cl:format cl:nil "box box~%~%================================================================================~%MSG: lidar_msgs/box~%float32 x_min~%float32 y_min~%float32 z_min~%float32 x_max~%float32 y_max~%float32 z_max~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BoxInfo)))
  "Returns full string definition for message of type 'BoxInfo"
  (cl:format cl:nil "box box~%~%================================================================================~%MSG: lidar_msgs/box~%float32 x_min~%float32 y_min~%float32 z_min~%float32 x_max~%float32 y_max~%float32 z_max~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BoxInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'box))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BoxInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'BoxInfo
    (cl:cons ':box (box msg))
))

; Auto-generated. Do not edit!


(cl:in-package lidar_msgs-msg)


;//! \htmlinclude SocketBox.msg.html

(cl:defclass <SocketBox> (roslisp-msg-protocol:ros-message)
  ((num_of_box
    :reader num_of_box
    :initarg :num_of_box
    :type cl:integer
    :initform 0)
   (box_info
    :reader box_info
    :initarg :box_info
    :type (cl:vector lidar_msgs-msg:BoxInfo)
   :initform (cl:make-array 0 :element-type 'lidar_msgs-msg:BoxInfo :initial-element (cl:make-instance 'lidar_msgs-msg:BoxInfo))))
)

(cl:defclass SocketBox (<SocketBox>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SocketBox>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SocketBox)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lidar_msgs-msg:<SocketBox> is deprecated: use lidar_msgs-msg:SocketBox instead.")))

(cl:ensure-generic-function 'num_of_box-val :lambda-list '(m))
(cl:defmethod num_of_box-val ((m <SocketBox>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_msgs-msg:num_of_box-val is deprecated.  Use lidar_msgs-msg:num_of_box instead.")
  (num_of_box m))

(cl:ensure-generic-function 'box_info-val :lambda-list '(m))
(cl:defmethod box_info-val ((m <SocketBox>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_msgs-msg:box_info-val is deprecated.  Use lidar_msgs-msg:box_info instead.")
  (box_info m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SocketBox>) ostream)
  "Serializes a message object of type '<SocketBox>"
  (cl:let* ((signed (cl:slot-value msg 'num_of_box)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'box_info))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'box_info))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SocketBox>) istream)
  "Deserializes a message object of type '<SocketBox>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_of_box) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'box_info) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'box_info)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'lidar_msgs-msg:BoxInfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SocketBox>)))
  "Returns string type for a message object of type '<SocketBox>"
  "lidar_msgs/SocketBox")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SocketBox)))
  "Returns string type for a message object of type 'SocketBox"
  "lidar_msgs/SocketBox")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SocketBox>)))
  "Returns md5sum for a message object of type '<SocketBox>"
  "7e7c1fed7bbc79bff735bfacc0aaebc3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SocketBox)))
  "Returns md5sum for a message object of type 'SocketBox"
  "7e7c1fed7bbc79bff735bfacc0aaebc3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SocketBox>)))
  "Returns full string definition for message of type '<SocketBox>"
  (cl:format cl:nil "int32 num_of_box~%BoxInfo[] box_info~%~%================================================================================~%MSG: lidar_msgs/BoxInfo~%box box~%~%================================================================================~%MSG: lidar_msgs/box~%float32 x_min~%float32 y_min~%float32 z_min~%float32 x_max~%float32 y_max~%float32 z_max~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SocketBox)))
  "Returns full string definition for message of type 'SocketBox"
  (cl:format cl:nil "int32 num_of_box~%BoxInfo[] box_info~%~%================================================================================~%MSG: lidar_msgs/BoxInfo~%box box~%~%================================================================================~%MSG: lidar_msgs/box~%float32 x_min~%float32 y_min~%float32 z_min~%float32 x_max~%float32 y_max~%float32 z_max~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SocketBox>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'box_info) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SocketBox>))
  "Converts a ROS message object to a list"
  (cl:list 'SocketBox
    (cl:cons ':num_of_box (num_of_box msg))
    (cl:cons ':box_info (box_info msg))
))

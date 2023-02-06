; Auto-generated. Do not edit!


(cl:in-package lidar_msgs-msg)


;//! \htmlinclude Curb.msg.html

(cl:defclass <Curb> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (points
    :reader points
    :initarg :points
    :type (cl:vector lidar_msgs-msg:PointStamped)
   :initform (cl:make-array 100 :element-type 'lidar_msgs-msg:PointStamped :initial-element (cl:make-instance 'lidar_msgs-msg:PointStamped))))
)

(cl:defclass Curb (<Curb>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Curb>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Curb)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lidar_msgs-msg:<Curb> is deprecated: use lidar_msgs-msg:Curb instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Curb>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_msgs-msg:header-val is deprecated.  Use lidar_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <Curb>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_msgs-msg:points-val is deprecated.  Use lidar_msgs-msg:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Curb>) ostream)
  "Serializes a message object of type '<Curb>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Curb>) istream)
  "Deserializes a message object of type '<Curb>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'points) (cl:make-array 100))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i 100)
    (cl:setf (cl:aref vals i) (cl:make-instance 'lidar_msgs-msg:PointStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Curb>)))
  "Returns string type for a message object of type '<Curb>"
  "lidar_msgs/Curb")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Curb)))
  "Returns string type for a message object of type 'Curb"
  "lidar_msgs/Curb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Curb>)))
  "Returns md5sum for a message object of type '<Curb>"
  "75af1c1affc9fe6f9de6b476e4c14f22")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Curb)))
  "Returns md5sum for a message object of type 'Curb"
  "75af1c1affc9fe6f9de6b476e4c14f22")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Curb>)))
  "Returns full string definition for message of type '<Curb>"
  (cl:format cl:nil "std_msgs/Header header~%PointStamped[100] points~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: lidar_msgs/PointStamped~%float64 x~%float64 y~%float64 z~%float64 intensity~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Curb)))
  "Returns full string definition for message of type 'Curb"
  (cl:format cl:nil "std_msgs/Header header~%PointStamped[100] points~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: lidar_msgs/PointStamped~%float64 x~%float64 y~%float64 z~%float64 intensity~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Curb>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Curb>))
  "Converts a ROS message object to a list"
  (cl:list 'Curb
    (cl:cons ':header (header msg))
    (cl:cons ':points (points msg))
))

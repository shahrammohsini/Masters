; Auto-generated. Do not edit!


(cl:in-package bionic_hand-msg)


;//! \htmlinclude FingerPos.msg.html

(cl:defclass <FingerPos> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (index
    :reader index
    :initarg :index
    :type bionic_hand-msg:FingerJoints
    :initform (cl:make-instance 'bionic_hand-msg:FingerJoints))
   (middle
    :reader middle
    :initarg :middle
    :type bionic_hand-msg:FingerJoints
    :initform (cl:make-instance 'bionic_hand-msg:FingerJoints)))
)

(cl:defclass FingerPos (<FingerPos>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FingerPos>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FingerPos)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bionic_hand-msg:<FingerPos> is deprecated: use bionic_hand-msg:FingerPos instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FingerPos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bionic_hand-msg:header-val is deprecated.  Use bionic_hand-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <FingerPos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bionic_hand-msg:index-val is deprecated.  Use bionic_hand-msg:index instead.")
  (index m))

(cl:ensure-generic-function 'middle-val :lambda-list '(m))
(cl:defmethod middle-val ((m <FingerPos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bionic_hand-msg:middle-val is deprecated.  Use bionic_hand-msg:middle instead.")
  (middle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FingerPos>) ostream)
  "Serializes a message object of type '<FingerPos>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'index) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'middle) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FingerPos>) istream)
  "Deserializes a message object of type '<FingerPos>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'index) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'middle) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FingerPos>)))
  "Returns string type for a message object of type '<FingerPos>"
  "bionic_hand/FingerPos")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FingerPos)))
  "Returns string type for a message object of type 'FingerPos"
  "bionic_hand/FingerPos")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FingerPos>)))
  "Returns md5sum for a message object of type '<FingerPos>"
  "df9ec1211fc81402f32ad35554d98a85")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FingerPos)))
  "Returns md5sum for a message object of type 'FingerPos"
  "df9ec1211fc81402f32ad35554d98a85")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FingerPos>)))
  "Returns full string definition for message of type '<FingerPos>"
  (cl:format cl:nil "std_msgs/Header header~%FingerJoints index~%FingerJoints middle~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: bionic_hand/FingerJoints~%float64 theta_M~%float64 theta_P~%float64 theta_D~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FingerPos)))
  "Returns full string definition for message of type 'FingerPos"
  (cl:format cl:nil "std_msgs/Header header~%FingerJoints index~%FingerJoints middle~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: bionic_hand/FingerJoints~%float64 theta_M~%float64 theta_P~%float64 theta_D~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FingerPos>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'index))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'middle))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FingerPos>))
  "Converts a ROS message object to a list"
  (cl:list 'FingerPos
    (cl:cons ':header (header msg))
    (cl:cons ':index (index msg))
    (cl:cons ':middle (middle msg))
))

; Auto-generated. Do not edit!


(cl:in-package bionic_hand-msg)


;//! \htmlinclude ControlCommands.msg.html

(cl:defclass <ControlCommands> (roslisp-msg-protocol:ros-message)
  ((PWM
    :reader PWM
    :initarg :PWM
    :type cl:float
    :initform 0.0))
)

(cl:defclass ControlCommands (<ControlCommands>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlCommands>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlCommands)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bionic_hand-msg:<ControlCommands> is deprecated: use bionic_hand-msg:ControlCommands instead.")))

(cl:ensure-generic-function 'PWM-val :lambda-list '(m))
(cl:defmethod PWM-val ((m <ControlCommands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bionic_hand-msg:PWM-val is deprecated.  Use bionic_hand-msg:PWM instead.")
  (PWM m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlCommands>) ostream)
  "Serializes a message object of type '<ControlCommands>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'PWM))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlCommands>) istream)
  "Deserializes a message object of type '<ControlCommands>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'PWM) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlCommands>)))
  "Returns string type for a message object of type '<ControlCommands>"
  "bionic_hand/ControlCommands")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlCommands)))
  "Returns string type for a message object of type 'ControlCommands"
  "bionic_hand/ControlCommands")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlCommands>)))
  "Returns md5sum for a message object of type '<ControlCommands>"
  "630d1348e66951f61746659ef3574616")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlCommands)))
  "Returns md5sum for a message object of type 'ControlCommands"
  "630d1348e66951f61746659ef3574616")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlCommands>)))
  "Returns full string definition for message of type '<ControlCommands>"
  (cl:format cl:nil "float64 PWM~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlCommands)))
  "Returns full string definition for message of type 'ControlCommands"
  (cl:format cl:nil "float64 PWM~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlCommands>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlCommands>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlCommands
    (cl:cons ':PWM (PWM msg))
))

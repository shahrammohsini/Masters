; Auto-generated. Do not edit!


(cl:in-package bionic_hand-msg)


;//! \htmlinclude SetPWM.msg.html

(cl:defclass <SetPWM> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (pwm
    :reader pwm
    :initarg :pwm
    :type cl:integer
    :initform 0))
)

(cl:defclass SetPWM (<SetPWM>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPWM>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPWM)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bionic_hand-msg:<SetPWM> is deprecated: use bionic_hand-msg:SetPWM instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <SetPWM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bionic_hand-msg:id-val is deprecated.  Use bionic_hand-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'pwm-val :lambda-list '(m))
(cl:defmethod pwm-val ((m <SetPWM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bionic_hand-msg:pwm-val is deprecated.  Use bionic_hand-msg:pwm instead.")
  (pwm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPWM>) ostream)
  "Serializes a message object of type '<SetPWM>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'pwm)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPWM>) istream)
  "Deserializes a message object of type '<SetPWM>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pwm) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPWM>)))
  "Returns string type for a message object of type '<SetPWM>"
  "bionic_hand/SetPWM")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPWM)))
  "Returns string type for a message object of type 'SetPWM"
  "bionic_hand/SetPWM")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPWM>)))
  "Returns md5sum for a message object of type '<SetPWM>"
  "7dcb9d04de9857b62a1d688544986dcd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPWM)))
  "Returns md5sum for a message object of type 'SetPWM"
  "7dcb9d04de9857b62a1d688544986dcd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPWM>)))
  "Returns full string definition for message of type '<SetPWM>"
  (cl:format cl:nil "uint8 id~%int32 pwm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPWM)))
  "Returns full string definition for message of type 'SetPWM"
  (cl:format cl:nil "uint8 id~%int32 pwm~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPWM>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPWM>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPWM
    (cl:cons ':id (id msg))
    (cl:cons ':pwm (pwm msg))
))

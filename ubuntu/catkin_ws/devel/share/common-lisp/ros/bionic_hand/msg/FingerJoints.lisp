; Auto-generated. Do not edit!


(cl:in-package bionic_hand-msg)


;//! \htmlinclude FingerJoints.msg.html

(cl:defclass <FingerJoints> (roslisp-msg-protocol:ros-message)
  ((theta_M
    :reader theta_M
    :initarg :theta_M
    :type cl:float
    :initform 0.0)
   (theta_P
    :reader theta_P
    :initarg :theta_P
    :type cl:float
    :initform 0.0)
   (theta_D
    :reader theta_D
    :initarg :theta_D
    :type cl:float
    :initform 0.0))
)

(cl:defclass FingerJoints (<FingerJoints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FingerJoints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FingerJoints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bionic_hand-msg:<FingerJoints> is deprecated: use bionic_hand-msg:FingerJoints instead.")))

(cl:ensure-generic-function 'theta_M-val :lambda-list '(m))
(cl:defmethod theta_M-val ((m <FingerJoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bionic_hand-msg:theta_M-val is deprecated.  Use bionic_hand-msg:theta_M instead.")
  (theta_M m))

(cl:ensure-generic-function 'theta_P-val :lambda-list '(m))
(cl:defmethod theta_P-val ((m <FingerJoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bionic_hand-msg:theta_P-val is deprecated.  Use bionic_hand-msg:theta_P instead.")
  (theta_P m))

(cl:ensure-generic-function 'theta_D-val :lambda-list '(m))
(cl:defmethod theta_D-val ((m <FingerJoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bionic_hand-msg:theta_D-val is deprecated.  Use bionic_hand-msg:theta_D instead.")
  (theta_D m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FingerJoints>) ostream)
  "Serializes a message object of type '<FingerJoints>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'theta_M))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'theta_P))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'theta_D))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FingerJoints>) istream)
  "Deserializes a message object of type '<FingerJoints>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta_M) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta_P) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta_D) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FingerJoints>)))
  "Returns string type for a message object of type '<FingerJoints>"
  "bionic_hand/FingerJoints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FingerJoints)))
  "Returns string type for a message object of type 'FingerJoints"
  "bionic_hand/FingerJoints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FingerJoints>)))
  "Returns md5sum for a message object of type '<FingerJoints>"
  "87bf9b04d1a94d2eda566ee32685c210")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FingerJoints)))
  "Returns md5sum for a message object of type 'FingerJoints"
  "87bf9b04d1a94d2eda566ee32685c210")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FingerJoints>)))
  "Returns full string definition for message of type '<FingerJoints>"
  (cl:format cl:nil "float64 theta_M~%float64 theta_P~%float64 theta_D~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FingerJoints)))
  "Returns full string definition for message of type 'FingerJoints"
  (cl:format cl:nil "float64 theta_M~%float64 theta_P~%float64 theta_D~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FingerJoints>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FingerJoints>))
  "Converts a ROS message object to a list"
  (cl:list 'FingerJoints
    (cl:cons ':theta_M (theta_M msg))
    (cl:cons ':theta_P (theta_P msg))
    (cl:cons ':theta_D (theta_D msg))
))

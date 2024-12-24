;; Auto-generated. Do not edit!


(when (boundp 'bionic_hand::FingerPos)
  (if (not (find-package "BIONIC_HAND"))
    (make-package "BIONIC_HAND"))
  (shadow 'FingerPos (find-package "BIONIC_HAND")))
(unless (find-package "BIONIC_HAND::FINGERPOS")
  (make-package "BIONIC_HAND::FINGERPOS"))

(in-package "ROS")
;;//! \htmlinclude FingerPos.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass bionic_hand::FingerPos
  :super ros::object
  :slots (_header _index _middle ))

(defmethod bionic_hand::FingerPos
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:index __index) (instance bionic_hand::FingerJoints :init))
    ((:middle __middle) (instance bionic_hand::FingerJoints :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _index __index)
   (setq _middle __middle)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:index
   (&rest __index)
   (if (keywordp (car __index))
       (send* _index __index)
     (progn
       (if __index (setq _index (car __index)))
       _index)))
  (:middle
   (&rest __middle)
   (if (keywordp (car __middle))
       (send* _middle __middle)
     (progn
       (if __middle (setq _middle (car __middle)))
       _middle)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bionic_hand/FingerJoints _index
    (send _index :serialization-length)
    ;; bionic_hand/FingerJoints _middle
    (send _middle :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bionic_hand/FingerJoints _index
       (send _index :serialize s)
     ;; bionic_hand/FingerJoints _middle
       (send _middle :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bionic_hand/FingerJoints _index
     (send _index :deserialize buf ptr-) (incf ptr- (send _index :serialization-length))
   ;; bionic_hand/FingerJoints _middle
     (send _middle :deserialize buf ptr-) (incf ptr- (send _middle :serialization-length))
   ;;
   self)
  )

(setf (get bionic_hand::FingerPos :md5sum-) "df9ec1211fc81402f32ad35554d98a85")
(setf (get bionic_hand::FingerPos :datatype-) "bionic_hand/FingerPos")
(setf (get bionic_hand::FingerPos :definition-)
      "std_msgs/Header header
FingerJoints index
FingerJoints middle
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
MSG: bionic_hand/FingerJoints
float64 theta_M
float64 theta_P
float64 theta_D
")



(provide :bionic_hand/FingerPos "df9ec1211fc81402f32ad35554d98a85")



;; Auto-generated. Do not edit!


(when (boundp 'bionic_hand::ControlCommands)
  (if (not (find-package "BIONIC_HAND"))
    (make-package "BIONIC_HAND"))
  (shadow 'ControlCommands (find-package "BIONIC_HAND")))
(unless (find-package "BIONIC_HAND::CONTROLCOMMANDS")
  (make-package "BIONIC_HAND::CONTROLCOMMANDS"))

(in-package "ROS")
;;//! \htmlinclude ControlCommands.msg.html


(defclass bionic_hand::ControlCommands
  :super ros::object
  :slots (_ID _PWM ))

(defmethod bionic_hand::ControlCommands
  (:init
   (&key
    ((:ID __ID) 0)
    ((:PWM __PWM) 0.0)
    )
   (send-super :init)
   (setq _ID (round __ID))
   (setq _PWM (float __PWM))
   self)
  (:ID
   (&optional __ID)
   (if __ID (setq _ID __ID)) _ID)
  (:PWM
   (&optional __PWM)
   (if __PWM (setq _PWM __PWM)) _PWM)
  (:serialization-length
   ()
   (+
    ;; int32 _ID
    4
    ;; float64 _PWM
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _ID
       (write-long _ID s)
     ;; float64 _PWM
       (sys::poke _PWM (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _ID
     (setq _ID (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _PWM
     (setq _PWM (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get bionic_hand::ControlCommands :md5sum-) "0710b5797f0353030d05fc8a9f52589b")
(setf (get bionic_hand::ControlCommands :datatype-) "bionic_hand/ControlCommands")
(setf (get bionic_hand::ControlCommands :definition-)
      "int32 ID
float64 PWM
")



(provide :bionic_hand/ControlCommands "0710b5797f0353030d05fc8a9f52589b")



;; Auto-generated. Do not edit!


(when (boundp 'bionic_hand::ControlCommand)
  (if (not (find-package "BIONIC_HAND"))
    (make-package "BIONIC_HAND"))
  (shadow 'ControlCommand (find-package "BIONIC_HAND")))
(unless (find-package "BIONIC_HAND::CONTROLCOMMAND")
  (make-package "BIONIC_HAND::CONTROLCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude ControlCommand.msg.html


(defclass bionic_hand::ControlCommand
  :super ros::object
  :slots (_PWM ))

(defmethod bionic_hand::ControlCommand
  (:init
   (&key
    ((:PWM __PWM) 0.0)
    )
   (send-super :init)
   (setq _PWM (float __PWM))
   self)
  (:PWM
   (&optional __PWM)
   (if __PWM (setq _PWM __PWM)) _PWM)
  (:serialization-length
   ()
   (+
    ;; float64 _PWM
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _PWM
       (sys::poke _PWM (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _PWM
     (setq _PWM (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get bionic_hand::ControlCommand :md5sum-) "630d1348e66951f61746659ef3574616")
(setf (get bionic_hand::ControlCommand :datatype-) "bionic_hand/ControlCommand")
(setf (get bionic_hand::ControlCommand :definition-)
      "float64 PWM
")



(provide :bionic_hand/ControlCommand "630d1348e66951f61746659ef3574616")



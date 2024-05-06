;; Auto-generated. Do not edit!


(when (boundp 'bionic_hand::SetPWM)
  (if (not (find-package "BIONIC_HAND"))
    (make-package "BIONIC_HAND"))
  (shadow 'SetPWM (find-package "BIONIC_HAND")))
(unless (find-package "BIONIC_HAND::SETPWM")
  (make-package "BIONIC_HAND::SETPWM"))

(in-package "ROS")
;;//! \htmlinclude SetPWM.msg.html


(defclass bionic_hand::SetPWM
  :super ros::object
  :slots (_id _pwm ))

(defmethod bionic_hand::SetPWM
  (:init
   (&key
    ((:id __id) 0)
    ((:pwm __pwm) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _pwm (round __pwm))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:pwm
   (&optional __pwm)
   (if __pwm (setq _pwm __pwm)) _pwm)
  (:serialization-length
   ()
   (+
    ;; uint8 _id
    1
    ;; int32 _pwm
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _id
       (write-byte _id s)
     ;; int32 _pwm
       (write-long _pwm s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int32 _pwm
     (setq _pwm (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get bionic_hand::SetPWM :md5sum-) "7dcb9d04de9857b62a1d688544986dcd")
(setf (get bionic_hand::SetPWM :datatype-) "bionic_hand/SetPWM")
(setf (get bionic_hand::SetPWM :definition-)
      "uint8 id
int32 pwm

")



(provide :bionic_hand/SetPWM "7dcb9d04de9857b62a1d688544986dcd")



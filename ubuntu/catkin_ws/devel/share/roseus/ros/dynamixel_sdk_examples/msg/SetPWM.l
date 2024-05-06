;; Auto-generated. Do not edit!


(when (boundp 'dynamixel_sdk_examples::SetPWM)
  (if (not (find-package "DYNAMIXEL_SDK_EXAMPLES"))
    (make-package "DYNAMIXEL_SDK_EXAMPLES"))
  (shadow 'SetPWM (find-package "DYNAMIXEL_SDK_EXAMPLES")))
(unless (find-package "DYNAMIXEL_SDK_EXAMPLES::SETPWM")
  (make-package "DYNAMIXEL_SDK_EXAMPLES::SETPWM"))

(in-package "ROS")
;;//! \htmlinclude SetPWM.msg.html


(defclass dynamixel_sdk_examples::SetPWM
  :super ros::object
  :slots (_id _pwm ))

(defmethod dynamixel_sdk_examples::SetPWM
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

(setf (get dynamixel_sdk_examples::SetPWM :md5sum-) "7dcb9d04de9857b62a1d688544986dcd")
(setf (get dynamixel_sdk_examples::SetPWM :datatype-) "dynamixel_sdk_examples/SetPWM")
(setf (get dynamixel_sdk_examples::SetPWM :definition-)
      "uint8 id
int32 pwm

")



(provide :dynamixel_sdk_examples/SetPWM "7dcb9d04de9857b62a1d688544986dcd")



;; Auto-generated. Do not edit!


(when (boundp 'bionic_hand::FingerJoints)
  (if (not (find-package "BIONIC_HAND"))
    (make-package "BIONIC_HAND"))
  (shadow 'FingerJoints (find-package "BIONIC_HAND")))
(unless (find-package "BIONIC_HAND::FINGERJOINTS")
  (make-package "BIONIC_HAND::FINGERJOINTS"))

(in-package "ROS")
;;//! \htmlinclude FingerJoints.msg.html


(defclass bionic_hand::FingerJoints
  :super ros::object
  :slots (_theta_M _theta_P _theta_D ))

(defmethod bionic_hand::FingerJoints
  (:init
   (&key
    ((:theta_M __theta_M) 0.0)
    ((:theta_P __theta_P) 0.0)
    ((:theta_D __theta_D) 0.0)
    )
   (send-super :init)
   (setq _theta_M (float __theta_M))
   (setq _theta_P (float __theta_P))
   (setq _theta_D (float __theta_D))
   self)
  (:theta_M
   (&optional __theta_M)
   (if __theta_M (setq _theta_M __theta_M)) _theta_M)
  (:theta_P
   (&optional __theta_P)
   (if __theta_P (setq _theta_P __theta_P)) _theta_P)
  (:theta_D
   (&optional __theta_D)
   (if __theta_D (setq _theta_D __theta_D)) _theta_D)
  (:serialization-length
   ()
   (+
    ;; float64 _theta_M
    8
    ;; float64 _theta_P
    8
    ;; float64 _theta_D
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _theta_M
       (sys::poke _theta_M (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta_P
       (sys::poke _theta_P (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta_D
       (sys::poke _theta_D (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _theta_M
     (setq _theta_M (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta_P
     (setq _theta_P (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta_D
     (setq _theta_D (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get bionic_hand::FingerJoints :md5sum-) "87bf9b04d1a94d2eda566ee32685c210")
(setf (get bionic_hand::FingerJoints :datatype-) "bionic_hand/FingerJoints")
(setf (get bionic_hand::FingerJoints :definition-)
      "float64 theta_M
float64 theta_P
float64 theta_D
")



(provide :bionic_hand/FingerJoints "87bf9b04d1a94d2eda566ee32685c210")



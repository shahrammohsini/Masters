;; Auto-generated. Do not edit!


(when (boundp 'bionic_hand::GetPosition)
  (if (not (find-package "BIONIC_HAND"))
    (make-package "BIONIC_HAND"))
  (shadow 'GetPosition (find-package "BIONIC_HAND")))
(unless (find-package "BIONIC_HAND::GETPOSITION")
  (make-package "BIONIC_HAND::GETPOSITION"))
(unless (find-package "BIONIC_HAND::GETPOSITIONREQUEST")
  (make-package "BIONIC_HAND::GETPOSITIONREQUEST"))
(unless (find-package "BIONIC_HAND::GETPOSITIONRESPONSE")
  (make-package "BIONIC_HAND::GETPOSITIONRESPONSE"))

(in-package "ROS")





(defclass bionic_hand::GetPositionRequest
  :super ros::object
  :slots (_id ))

(defmethod bionic_hand::GetPositionRequest
  (:init
   (&key
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; uint8 _id
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _id
       (write-byte _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass bionic_hand::GetPositionResponse
  :super ros::object
  :slots (_position ))

(defmethod bionic_hand::GetPositionResponse
  (:init
   (&key
    ((:position __position) 0)
    )
   (send-super :init)
   (setq _position (round __position))
   self)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:serialization-length
   ()
   (+
    ;; int32 _position
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _position
       (write-long _position s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _position
     (setq _position (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass bionic_hand::GetPosition
  :super ros::object
  :slots ())

(setf (get bionic_hand::GetPosition :md5sum-) "b532ace3b383dc4c9e64687156423ac0")
(setf (get bionic_hand::GetPosition :datatype-) "bionic_hand/GetPosition")
(setf (get bionic_hand::GetPosition :request) bionic_hand::GetPositionRequest)
(setf (get bionic_hand::GetPosition :response) bionic_hand::GetPositionResponse)

(defmethod bionic_hand::GetPositionRequest
  (:response () (instance bionic_hand::GetPositionResponse :init)))

(setf (get bionic_hand::GetPositionRequest :md5sum-) "b532ace3b383dc4c9e64687156423ac0")
(setf (get bionic_hand::GetPositionRequest :datatype-) "bionic_hand/GetPositionRequest")
(setf (get bionic_hand::GetPositionRequest :definition-)
      "uint8 id
---
int32 position

")

(setf (get bionic_hand::GetPositionResponse :md5sum-) "b532ace3b383dc4c9e64687156423ac0")
(setf (get bionic_hand::GetPositionResponse :datatype-) "bionic_hand/GetPositionResponse")
(setf (get bionic_hand::GetPositionResponse :definition-)
      "uint8 id
---
int32 position

")



(provide :bionic_hand/GetPosition "b532ace3b383dc4c9e64687156423ac0")



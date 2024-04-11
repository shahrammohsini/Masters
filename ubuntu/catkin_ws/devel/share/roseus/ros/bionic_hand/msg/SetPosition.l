;; Auto-generated. Do not edit!


(when (boundp 'bionic_hand::SetPosition)
  (if (not (find-package "BIONIC_HAND"))
    (make-package "BIONIC_HAND"))
  (shadow 'SetPosition (find-package "BIONIC_HAND")))
(unless (find-package "BIONIC_HAND::SETPOSITION")
  (make-package "BIONIC_HAND::SETPOSITION"))

(in-package "ROS")
;;//! \htmlinclude SetPosition.msg.html


(defclass bionic_hand::SetPosition
  :super ros::object
  :slots (_id _position ))

(defmethod bionic_hand::SetPosition
  (:init
   (&key
    ((:id __id) 0)
    ((:position __position) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _position (round __position))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:serialization-length
   ()
   (+
    ;; uint8 _id
    1
    ;; int32 _position
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _id
       (write-byte _id s)
     ;; int32 _position
       (write-long _position s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int32 _position
     (setq _position (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get bionic_hand::SetPosition :md5sum-) "0a775458729eb3272bc88b4f5f764cc8")
(setf (get bionic_hand::SetPosition :datatype-) "bionic_hand/SetPosition")
(setf (get bionic_hand::SetPosition :definition-)
      "uint8 id
int32 position

")



(provide :bionic_hand/SetPosition "0a775458729eb3272bc88b4f5f764cc8")



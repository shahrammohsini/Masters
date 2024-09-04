
(cl:in-package :asdf)

(defsystem "bionic_hand-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "BulkSetItem" :depends-on ("_package_BulkSetItem"))
    (:file "_package_BulkSetItem" :depends-on ("_package"))
    (:file "ControlCommands" :depends-on ("_package_ControlCommands"))
    (:file "_package_ControlCommands" :depends-on ("_package"))
    (:file "FingerPos" :depends-on ("_package_FingerPos"))
    (:file "_package_FingerPos" :depends-on ("_package"))
    (:file "SetPWM" :depends-on ("_package_SetPWM"))
    (:file "_package_SetPWM" :depends-on ("_package"))
    (:file "SetPosition" :depends-on ("_package_SetPosition"))
    (:file "_package_SetPosition" :depends-on ("_package"))
    (:file "SyncSetPosition" :depends-on ("_package_SyncSetPosition"))
    (:file "_package_SyncSetPosition" :depends-on ("_package"))
  ))
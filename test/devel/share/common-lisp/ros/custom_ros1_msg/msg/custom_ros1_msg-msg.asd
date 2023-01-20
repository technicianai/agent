
(cl:in-package :asdf)

(defsystem "custom_ros1_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CustomMessageExample" :depends-on ("_package_CustomMessageExample"))
    (:file "_package_CustomMessageExample" :depends-on ("_package"))
  ))
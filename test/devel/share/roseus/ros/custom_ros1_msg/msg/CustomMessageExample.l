;; Auto-generated. Do not edit!


(when (boundp 'custom_ros1_msg::CustomMessageExample)
  (if (not (find-package "CUSTOM_ROS1_MSG"))
    (make-package "CUSTOM_ROS1_MSG"))
  (shadow 'CustomMessageExample (find-package "CUSTOM_ROS1_MSG")))
(unless (find-package "CUSTOM_ROS1_MSG::CUSTOMMESSAGEEXAMPLE")
  (make-package "CUSTOM_ROS1_MSG::CUSTOMMESSAGEEXAMPLE"))

(in-package "ROS")
;;//! \htmlinclude CustomMessageExample.msg.html


(defclass custom_ros1_msg::CustomMessageExample
  :super ros::object
  :slots (_name _age ))

(defmethod custom_ros1_msg::CustomMessageExample
  (:init
   (&key
    ((:name __name) "")
    ((:age __age) 0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _age (round __age))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:age
   (&optional __age)
   (if __age (setq _age __age)) _age)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; int32 _age
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; int32 _age
       (write-long _age s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _age
     (setq _age (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get custom_ros1_msg::CustomMessageExample :md5sum-) "83da5ca00b8e049b0559653a472c88a5")
(setf (get custom_ros1_msg::CustomMessageExample :datatype-) "custom_ros1_msg/CustomMessageExample")
(setf (get custom_ros1_msg::CustomMessageExample :definition-)
      "string name
int32 age

")



(provide :custom_ros1_msg/CustomMessageExample "83da5ca00b8e049b0559653a472c88a5")



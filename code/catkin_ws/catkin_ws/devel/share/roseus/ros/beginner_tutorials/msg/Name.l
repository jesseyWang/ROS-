;; Auto-generated. Do not edit!


(when (boundp 'beginner_tutorials::Name)
  (if (not (find-package "BEGINNER_TUTORIALS"))
    (make-package "BEGINNER_TUTORIALS"))
  (shadow 'Name (find-package "BEGINNER_TUTORIALS")))
(unless (find-package "BEGINNER_TUTORIALS::NAME")
  (make-package "BEGINNER_TUTORIALS::NAME"))

(in-package "ROS")
;;//! \htmlinclude Name.msg.html


(defclass beginner_tutorials::Name
  :super ros::object
  :slots (_name ))

(defmethod beginner_tutorials::Name
  (:init
   (&key
    ((:name __name) "")
    )
   (send-super :init)
   (setq _name (string __name))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get beginner_tutorials::Name :md5sum-) "c1f3d28f1b044c871e6eff2e9fc3c667")
(setf (get beginner_tutorials::Name :datatype-) "beginner_tutorials/Name")
(setf (get beginner_tutorials::Name :definition-)
      "string name
")



(provide :beginner_tutorials/Name "c1f3d28f1b044c871e6eff2e9fc3c667")



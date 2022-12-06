; Auto-generated. Do not edit!


(cl:in-package beginner_tutorials-msg)


;//! \htmlinclude Name.msg.html

(cl:defclass <Name> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass Name (<Name>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Name>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Name)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beginner_tutorials-msg:<Name> is deprecated: use beginner_tutorials-msg:Name instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <Name>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:name-val is deprecated.  Use beginner_tutorials-msg:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Name>) ostream)
  "Serializes a message object of type '<Name>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Name>) istream)
  "Deserializes a message object of type '<Name>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Name>)))
  "Returns string type for a message object of type '<Name>"
  "beginner_tutorials/Name")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Name)))
  "Returns string type for a message object of type 'Name"
  "beginner_tutorials/Name")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Name>)))
  "Returns md5sum for a message object of type '<Name>"
  "c1f3d28f1b044c871e6eff2e9fc3c667")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Name)))
  "Returns md5sum for a message object of type 'Name"
  "c1f3d28f1b044c871e6eff2e9fc3c667")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Name>)))
  "Returns full string definition for message of type '<Name>"
  (cl:format cl:nil "string name~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Name)))
  "Returns full string definition for message of type 'Name"
  (cl:format cl:nil "string name~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Name>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Name>))
  "Converts a ROS message object to a list"
  (cl:list 'Name
    (cl:cons ':name (name msg))
))

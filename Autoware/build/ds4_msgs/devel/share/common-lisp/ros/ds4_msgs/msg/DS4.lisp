; Auto-generated. Do not edit!


(cl:in-package ds4_msgs-msg)


;//! \htmlinclude DS4.msg.html

(cl:defclass <DS4> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (square
    :reader square
    :initarg :square
    :type cl:boolean
    :initform cl:nil)
   (cross
    :reader cross
    :initarg :cross
    :type cl:boolean
    :initform cl:nil)
   (circle
    :reader circle
    :initarg :circle
    :type cl:boolean
    :initform cl:nil)
   (triangle
    :reader triangle
    :initarg :triangle
    :type cl:boolean
    :initform cl:nil)
   (up
    :reader up
    :initarg :up
    :type cl:boolean
    :initform cl:nil)
   (right
    :reader right
    :initarg :right
    :type cl:boolean
    :initform cl:nil)
   (down
    :reader down
    :initarg :down
    :type cl:boolean
    :initform cl:nil)
   (left
    :reader left
    :initarg :left
    :type cl:boolean
    :initform cl:nil)
   (l1
    :reader l1
    :initarg :l1
    :type cl:boolean
    :initform cl:nil)
   (r1
    :reader r1
    :initarg :r1
    :type cl:boolean
    :initform cl:nil)
   (l2
    :reader l2
    :initarg :l2
    :type cl:float
    :initform 0.0)
   (r2
    :reader r2
    :initarg :r2
    :type cl:float
    :initform 0.0)
   (l3
    :reader l3
    :initarg :l3
    :type cl:boolean
    :initform cl:nil)
   (r3
    :reader r3
    :initarg :r3
    :type cl:boolean
    :initform cl:nil)
   (left_x
    :reader left_x
    :initarg :left_x
    :type cl:float
    :initform 0.0)
   (left_y
    :reader left_y
    :initarg :left_y
    :type cl:float
    :initform 0.0)
   (right_x
    :reader right_x
    :initarg :right_x
    :type cl:float
    :initform 0.0)
   (right_y
    :reader right_y
    :initarg :right_y
    :type cl:float
    :initform 0.0)
   (share
    :reader share
    :initarg :share
    :type cl:boolean
    :initform cl:nil)
   (option
    :reader option
    :initarg :option
    :type cl:boolean
    :initform cl:nil)
   (ps
    :reader ps
    :initarg :ps
    :type cl:boolean
    :initform cl:nil)
   (touchpad
    :reader touchpad
    :initarg :touchpad
    :type cl:boolean
    :initform cl:nil)
   (acc_x
    :reader acc_x
    :initarg :acc_x
    :type cl:float
    :initform 0.0)
   (acc_y
    :reader acc_y
    :initarg :acc_y
    :type cl:float
    :initform 0.0)
   (acc_z
    :reader acc_z
    :initarg :acc_z
    :type cl:float
    :initform 0.0)
   (gyro_rol
    :reader gyro_rol
    :initarg :gyro_rol
    :type cl:float
    :initform 0.0)
   (gyro_pit
    :reader gyro_pit
    :initarg :gyro_pit
    :type cl:float
    :initform 0.0)
   (gyro_yaw
    :reader gyro_yaw
    :initarg :gyro_yaw
    :type cl:float
    :initform 0.0)
   (connected
    :reader connected
    :initarg :connected
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DS4 (<DS4>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DS4>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DS4)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ds4_msgs-msg:<DS4> is deprecated: use ds4_msgs-msg:DS4 instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:header-val is deprecated.  Use ds4_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'square-val :lambda-list '(m))
(cl:defmethod square-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:square-val is deprecated.  Use ds4_msgs-msg:square instead.")
  (square m))

(cl:ensure-generic-function 'cross-val :lambda-list '(m))
(cl:defmethod cross-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:cross-val is deprecated.  Use ds4_msgs-msg:cross instead.")
  (cross m))

(cl:ensure-generic-function 'circle-val :lambda-list '(m))
(cl:defmethod circle-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:circle-val is deprecated.  Use ds4_msgs-msg:circle instead.")
  (circle m))

(cl:ensure-generic-function 'triangle-val :lambda-list '(m))
(cl:defmethod triangle-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:triangle-val is deprecated.  Use ds4_msgs-msg:triangle instead.")
  (triangle m))

(cl:ensure-generic-function 'up-val :lambda-list '(m))
(cl:defmethod up-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:up-val is deprecated.  Use ds4_msgs-msg:up instead.")
  (up m))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:right-val is deprecated.  Use ds4_msgs-msg:right instead.")
  (right m))

(cl:ensure-generic-function 'down-val :lambda-list '(m))
(cl:defmethod down-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:down-val is deprecated.  Use ds4_msgs-msg:down instead.")
  (down m))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:left-val is deprecated.  Use ds4_msgs-msg:left instead.")
  (left m))

(cl:ensure-generic-function 'l1-val :lambda-list '(m))
(cl:defmethod l1-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:l1-val is deprecated.  Use ds4_msgs-msg:l1 instead.")
  (l1 m))

(cl:ensure-generic-function 'r1-val :lambda-list '(m))
(cl:defmethod r1-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:r1-val is deprecated.  Use ds4_msgs-msg:r1 instead.")
  (r1 m))

(cl:ensure-generic-function 'l2-val :lambda-list '(m))
(cl:defmethod l2-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:l2-val is deprecated.  Use ds4_msgs-msg:l2 instead.")
  (l2 m))

(cl:ensure-generic-function 'r2-val :lambda-list '(m))
(cl:defmethod r2-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:r2-val is deprecated.  Use ds4_msgs-msg:r2 instead.")
  (r2 m))

(cl:ensure-generic-function 'l3-val :lambda-list '(m))
(cl:defmethod l3-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:l3-val is deprecated.  Use ds4_msgs-msg:l3 instead.")
  (l3 m))

(cl:ensure-generic-function 'r3-val :lambda-list '(m))
(cl:defmethod r3-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:r3-val is deprecated.  Use ds4_msgs-msg:r3 instead.")
  (r3 m))

(cl:ensure-generic-function 'left_x-val :lambda-list '(m))
(cl:defmethod left_x-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:left_x-val is deprecated.  Use ds4_msgs-msg:left_x instead.")
  (left_x m))

(cl:ensure-generic-function 'left_y-val :lambda-list '(m))
(cl:defmethod left_y-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:left_y-val is deprecated.  Use ds4_msgs-msg:left_y instead.")
  (left_y m))

(cl:ensure-generic-function 'right_x-val :lambda-list '(m))
(cl:defmethod right_x-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:right_x-val is deprecated.  Use ds4_msgs-msg:right_x instead.")
  (right_x m))

(cl:ensure-generic-function 'right_y-val :lambda-list '(m))
(cl:defmethod right_y-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:right_y-val is deprecated.  Use ds4_msgs-msg:right_y instead.")
  (right_y m))

(cl:ensure-generic-function 'share-val :lambda-list '(m))
(cl:defmethod share-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:share-val is deprecated.  Use ds4_msgs-msg:share instead.")
  (share m))

(cl:ensure-generic-function 'option-val :lambda-list '(m))
(cl:defmethod option-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:option-val is deprecated.  Use ds4_msgs-msg:option instead.")
  (option m))

(cl:ensure-generic-function 'ps-val :lambda-list '(m))
(cl:defmethod ps-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:ps-val is deprecated.  Use ds4_msgs-msg:ps instead.")
  (ps m))

(cl:ensure-generic-function 'touchpad-val :lambda-list '(m))
(cl:defmethod touchpad-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:touchpad-val is deprecated.  Use ds4_msgs-msg:touchpad instead.")
  (touchpad m))

(cl:ensure-generic-function 'acc_x-val :lambda-list '(m))
(cl:defmethod acc_x-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:acc_x-val is deprecated.  Use ds4_msgs-msg:acc_x instead.")
  (acc_x m))

(cl:ensure-generic-function 'acc_y-val :lambda-list '(m))
(cl:defmethod acc_y-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:acc_y-val is deprecated.  Use ds4_msgs-msg:acc_y instead.")
  (acc_y m))

(cl:ensure-generic-function 'acc_z-val :lambda-list '(m))
(cl:defmethod acc_z-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:acc_z-val is deprecated.  Use ds4_msgs-msg:acc_z instead.")
  (acc_z m))

(cl:ensure-generic-function 'gyro_rol-val :lambda-list '(m))
(cl:defmethod gyro_rol-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:gyro_rol-val is deprecated.  Use ds4_msgs-msg:gyro_rol instead.")
  (gyro_rol m))

(cl:ensure-generic-function 'gyro_pit-val :lambda-list '(m))
(cl:defmethod gyro_pit-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:gyro_pit-val is deprecated.  Use ds4_msgs-msg:gyro_pit instead.")
  (gyro_pit m))

(cl:ensure-generic-function 'gyro_yaw-val :lambda-list '(m))
(cl:defmethod gyro_yaw-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:gyro_yaw-val is deprecated.  Use ds4_msgs-msg:gyro_yaw instead.")
  (gyro_yaw m))

(cl:ensure-generic-function 'connected-val :lambda-list '(m))
(cl:defmethod connected-val ((m <DS4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ds4_msgs-msg:connected-val is deprecated.  Use ds4_msgs-msg:connected instead.")
  (connected m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DS4>) ostream)
  "Serializes a message object of type '<DS4>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'square) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cross) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'circle) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'triangle) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'up) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'right) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'down) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'left) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'l1) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'r1) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'l2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'r2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'l3) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'r3) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'left_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'left_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'right_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'right_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'share) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'option) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ps) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'touchpad) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'acc_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'acc_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'acc_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'gyro_rol))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'gyro_pit))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'gyro_yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'connected) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DS4>) istream)
  "Deserializes a message object of type '<DS4>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'square) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cross) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'circle) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'triangle) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'up) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'right) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'down) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'left) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'l1) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'r1) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'l2) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'r2) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'l3) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'r3) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'share) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'option) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ps) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'touchpad) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'acc_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'acc_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'acc_z) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gyro_rol) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gyro_pit) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gyro_yaw) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'connected) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DS4>)))
  "Returns string type for a message object of type '<DS4>"
  "ds4_msgs/DS4")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DS4)))
  "Returns string type for a message object of type 'DS4"
  "ds4_msgs/DS4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DS4>)))
  "Returns md5sum for a message object of type '<DS4>"
  "9c17d8a8df3d9463ab329c13d2358031")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DS4)))
  "Returns md5sum for a message object of type 'DS4"
  "9c17d8a8df3d9463ab329c13d2358031")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DS4>)))
  "Returns full string definition for message of type '<DS4>"
  (cl:format cl:nil "Header header~%~%bool square~%bool cross~%bool circle~%bool triangle~%~%bool up~%bool right~%bool down~%bool left~%~%bool l1~%bool r1~%float64 l2~%float64 r2~%bool l3~%bool r3~%~%float64 left_x~%float64 left_y~%float64 right_x~%float64 right_y~%~%bool share~%bool option~%bool ps~%bool touchpad~%~%float64 acc_x~%float64 acc_y~%float64 acc_z~%~%float64 gyro_rol~%float64 gyro_pit~%float64 gyro_yaw~%~%bool connected~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DS4)))
  "Returns full string definition for message of type 'DS4"
  (cl:format cl:nil "Header header~%~%bool square~%bool cross~%bool circle~%bool triangle~%~%bool up~%bool right~%bool down~%bool left~%~%bool l1~%bool r1~%float64 l2~%float64 r2~%bool l3~%bool r3~%~%float64 left_x~%float64 left_y~%float64 right_x~%float64 right_y~%~%bool share~%bool option~%bool ps~%bool touchpad~%~%float64 acc_x~%float64 acc_y~%float64 acc_z~%~%float64 gyro_rol~%float64 gyro_pit~%float64 gyro_yaw~%~%bool connected~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DS4>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     8
     8
     1
     1
     8
     8
     8
     8
     1
     1
     1
     1
     8
     8
     8
     8
     8
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DS4>))
  "Converts a ROS message object to a list"
  (cl:list 'DS4
    (cl:cons ':header (header msg))
    (cl:cons ':square (square msg))
    (cl:cons ':cross (cross msg))
    (cl:cons ':circle (circle msg))
    (cl:cons ':triangle (triangle msg))
    (cl:cons ':up (up msg))
    (cl:cons ':right (right msg))
    (cl:cons ':down (down msg))
    (cl:cons ':left (left msg))
    (cl:cons ':l1 (l1 msg))
    (cl:cons ':r1 (r1 msg))
    (cl:cons ':l2 (l2 msg))
    (cl:cons ':r2 (r2 msg))
    (cl:cons ':l3 (l3 msg))
    (cl:cons ':r3 (r3 msg))
    (cl:cons ':left_x (left_x msg))
    (cl:cons ':left_y (left_y msg))
    (cl:cons ':right_x (right_x msg))
    (cl:cons ':right_y (right_y msg))
    (cl:cons ':share (share msg))
    (cl:cons ':option (option msg))
    (cl:cons ':ps (ps msg))
    (cl:cons ':touchpad (touchpad msg))
    (cl:cons ':acc_x (acc_x msg))
    (cl:cons ':acc_y (acc_y msg))
    (cl:cons ':acc_z (acc_z msg))
    (cl:cons ':gyro_rol (gyro_rol msg))
    (cl:cons ':gyro_pit (gyro_pit msg))
    (cl:cons ':gyro_yaw (gyro_yaw msg))
    (cl:cons ':connected (connected msg))
))

;; Auto-generated. Do not edit!


(when (boundp 'ds4_msgs::DS4)
  (if (not (find-package "DS4_MSGS"))
    (make-package "DS4_MSGS"))
  (shadow 'DS4 (find-package "DS4_MSGS")))
(unless (find-package "DS4_MSGS::DS4")
  (make-package "DS4_MSGS::DS4"))

(in-package "ROS")
;;//! \htmlinclude DS4.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ds4_msgs::DS4
  :super ros::object
  :slots (_header _square _cross _circle _triangle _up _right _down _left _l1 _r1 _l2 _r2 _l3 _r3 _left_x _left_y _right_x _right_y _share _option _ps _touchpad _acc_x _acc_y _acc_z _gyro_rol _gyro_pit _gyro_yaw _connected ))

(defmethod ds4_msgs::DS4
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:square __square) nil)
    ((:cross __cross) nil)
    ((:circle __circle) nil)
    ((:triangle __triangle) nil)
    ((:up __up) nil)
    ((:right __right) nil)
    ((:down __down) nil)
    ((:left __left) nil)
    ((:l1 __l1) nil)
    ((:r1 __r1) nil)
    ((:l2 __l2) 0.0)
    ((:r2 __r2) 0.0)
    ((:l3 __l3) nil)
    ((:r3 __r3) nil)
    ((:left_x __left_x) 0.0)
    ((:left_y __left_y) 0.0)
    ((:right_x __right_x) 0.0)
    ((:right_y __right_y) 0.0)
    ((:share __share) nil)
    ((:option __option) nil)
    ((:ps __ps) nil)
    ((:touchpad __touchpad) nil)
    ((:acc_x __acc_x) 0.0)
    ((:acc_y __acc_y) 0.0)
    ((:acc_z __acc_z) 0.0)
    ((:gyro_rol __gyro_rol) 0.0)
    ((:gyro_pit __gyro_pit) 0.0)
    ((:gyro_yaw __gyro_yaw) 0.0)
    ((:connected __connected) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _square __square)
   (setq _cross __cross)
   (setq _circle __circle)
   (setq _triangle __triangle)
   (setq _up __up)
   (setq _right __right)
   (setq _down __down)
   (setq _left __left)
   (setq _l1 __l1)
   (setq _r1 __r1)
   (setq _l2 (float __l2))
   (setq _r2 (float __r2))
   (setq _l3 __l3)
   (setq _r3 __r3)
   (setq _left_x (float __left_x))
   (setq _left_y (float __left_y))
   (setq _right_x (float __right_x))
   (setq _right_y (float __right_y))
   (setq _share __share)
   (setq _option __option)
   (setq _ps __ps)
   (setq _touchpad __touchpad)
   (setq _acc_x (float __acc_x))
   (setq _acc_y (float __acc_y))
   (setq _acc_z (float __acc_z))
   (setq _gyro_rol (float __gyro_rol))
   (setq _gyro_pit (float __gyro_pit))
   (setq _gyro_yaw (float __gyro_yaw))
   (setq _connected __connected)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:square
   (&optional __square)
   (if __square (setq _square __square)) _square)
  (:cross
   (&optional __cross)
   (if __cross (setq _cross __cross)) _cross)
  (:circle
   (&optional __circle)
   (if __circle (setq _circle __circle)) _circle)
  (:triangle
   (&optional __triangle)
   (if __triangle (setq _triangle __triangle)) _triangle)
  (:up
   (&optional __up)
   (if __up (setq _up __up)) _up)
  (:right
   (&optional __right)
   (if __right (setq _right __right)) _right)
  (:down
   (&optional __down)
   (if __down (setq _down __down)) _down)
  (:left
   (&optional __left)
   (if __left (setq _left __left)) _left)
  (:l1
   (&optional __l1)
   (if __l1 (setq _l1 __l1)) _l1)
  (:r1
   (&optional __r1)
   (if __r1 (setq _r1 __r1)) _r1)
  (:l2
   (&optional __l2)
   (if __l2 (setq _l2 __l2)) _l2)
  (:r2
   (&optional __r2)
   (if __r2 (setq _r2 __r2)) _r2)
  (:l3
   (&optional __l3)
   (if __l3 (setq _l3 __l3)) _l3)
  (:r3
   (&optional __r3)
   (if __r3 (setq _r3 __r3)) _r3)
  (:left_x
   (&optional __left_x)
   (if __left_x (setq _left_x __left_x)) _left_x)
  (:left_y
   (&optional __left_y)
   (if __left_y (setq _left_y __left_y)) _left_y)
  (:right_x
   (&optional __right_x)
   (if __right_x (setq _right_x __right_x)) _right_x)
  (:right_y
   (&optional __right_y)
   (if __right_y (setq _right_y __right_y)) _right_y)
  (:share
   (&optional __share)
   (if __share (setq _share __share)) _share)
  (:option
   (&optional __option)
   (if __option (setq _option __option)) _option)
  (:ps
   (&optional __ps)
   (if __ps (setq _ps __ps)) _ps)
  (:touchpad
   (&optional __touchpad)
   (if __touchpad (setq _touchpad __touchpad)) _touchpad)
  (:acc_x
   (&optional __acc_x)
   (if __acc_x (setq _acc_x __acc_x)) _acc_x)
  (:acc_y
   (&optional __acc_y)
   (if __acc_y (setq _acc_y __acc_y)) _acc_y)
  (:acc_z
   (&optional __acc_z)
   (if __acc_z (setq _acc_z __acc_z)) _acc_z)
  (:gyro_rol
   (&optional __gyro_rol)
   (if __gyro_rol (setq _gyro_rol __gyro_rol)) _gyro_rol)
  (:gyro_pit
   (&optional __gyro_pit)
   (if __gyro_pit (setq _gyro_pit __gyro_pit)) _gyro_pit)
  (:gyro_yaw
   (&optional __gyro_yaw)
   (if __gyro_yaw (setq _gyro_yaw __gyro_yaw)) _gyro_yaw)
  (:connected
   (&optional __connected)
   (if __connected (setq _connected __connected)) _connected)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _square
    1
    ;; bool _cross
    1
    ;; bool _circle
    1
    ;; bool _triangle
    1
    ;; bool _up
    1
    ;; bool _right
    1
    ;; bool _down
    1
    ;; bool _left
    1
    ;; bool _l1
    1
    ;; bool _r1
    1
    ;; float64 _l2
    8
    ;; float64 _r2
    8
    ;; bool _l3
    1
    ;; bool _r3
    1
    ;; float64 _left_x
    8
    ;; float64 _left_y
    8
    ;; float64 _right_x
    8
    ;; float64 _right_y
    8
    ;; bool _share
    1
    ;; bool _option
    1
    ;; bool _ps
    1
    ;; bool _touchpad
    1
    ;; float64 _acc_x
    8
    ;; float64 _acc_y
    8
    ;; float64 _acc_z
    8
    ;; float64 _gyro_rol
    8
    ;; float64 _gyro_pit
    8
    ;; float64 _gyro_yaw
    8
    ;; bool _connected
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _square
       (if _square (write-byte -1 s) (write-byte 0 s))
     ;; bool _cross
       (if _cross (write-byte -1 s) (write-byte 0 s))
     ;; bool _circle
       (if _circle (write-byte -1 s) (write-byte 0 s))
     ;; bool _triangle
       (if _triangle (write-byte -1 s) (write-byte 0 s))
     ;; bool _up
       (if _up (write-byte -1 s) (write-byte 0 s))
     ;; bool _right
       (if _right (write-byte -1 s) (write-byte 0 s))
     ;; bool _down
       (if _down (write-byte -1 s) (write-byte 0 s))
     ;; bool _left
       (if _left (write-byte -1 s) (write-byte 0 s))
     ;; bool _l1
       (if _l1 (write-byte -1 s) (write-byte 0 s))
     ;; bool _r1
       (if _r1 (write-byte -1 s) (write-byte 0 s))
     ;; float64 _l2
       (sys::poke _l2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _r2
       (sys::poke _r2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _l3
       (if _l3 (write-byte -1 s) (write-byte 0 s))
     ;; bool _r3
       (if _r3 (write-byte -1 s) (write-byte 0 s))
     ;; float64 _left_x
       (sys::poke _left_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _left_y
       (sys::poke _left_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right_x
       (sys::poke _right_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right_y
       (sys::poke _right_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _share
       (if _share (write-byte -1 s) (write-byte 0 s))
     ;; bool _option
       (if _option (write-byte -1 s) (write-byte 0 s))
     ;; bool _ps
       (if _ps (write-byte -1 s) (write-byte 0 s))
     ;; bool _touchpad
       (if _touchpad (write-byte -1 s) (write-byte 0 s))
     ;; float64 _acc_x
       (sys::poke _acc_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _acc_y
       (sys::poke _acc_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _acc_z
       (sys::poke _acc_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _gyro_rol
       (sys::poke _gyro_rol (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _gyro_pit
       (sys::poke _gyro_pit (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _gyro_yaw
       (sys::poke _gyro_yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _connected
       (if _connected (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _square
     (setq _square (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _cross
     (setq _cross (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _circle
     (setq _circle (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _triangle
     (setq _triangle (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _up
     (setq _up (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _right
     (setq _right (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _down
     (setq _down (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _left
     (setq _left (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _l1
     (setq _l1 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _r1
     (setq _r1 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _l2
     (setq _l2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _r2
     (setq _r2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _l3
     (setq _l3 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _r3
     (setq _r3 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _left_x
     (setq _left_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _left_y
     (setq _left_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right_x
     (setq _right_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right_y
     (setq _right_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _share
     (setq _share (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _option
     (setq _option (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ps
     (setq _ps (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _touchpad
     (setq _touchpad (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _acc_x
     (setq _acc_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _acc_y
     (setq _acc_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _acc_z
     (setq _acc_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _gyro_rol
     (setq _gyro_rol (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _gyro_pit
     (setq _gyro_pit (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _gyro_yaw
     (setq _gyro_yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _connected
     (setq _connected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get ds4_msgs::DS4 :md5sum-) "9c17d8a8df3d9463ab329c13d2358031")
(setf (get ds4_msgs::DS4 :datatype-) "ds4_msgs/DS4")
(setf (get ds4_msgs::DS4 :definition-)
      "Header header

bool square
bool cross
bool circle
bool triangle

bool up
bool right
bool down
bool left

bool l1
bool r1
float64 l2
float64 r2
bool l3
bool r3

float64 left_x
float64 left_y
float64 right_x
float64 right_y

bool share
bool option
bool ps
bool touchpad

float64 acc_x
float64 acc_y
float64 acc_z

float64 gyro_rol
float64 gyro_pit
float64 gyro_yaw

bool connected

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :ds4_msgs/DS4 "9c17d8a8df3d9463ab329c13d2358031")



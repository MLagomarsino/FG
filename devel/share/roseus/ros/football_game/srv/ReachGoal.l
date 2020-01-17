;; Auto-generated. Do not edit!


(when (boundp 'football_game::ReachGoal)
  (if (not (find-package "FOOTBALL_GAME"))
    (make-package "FOOTBALL_GAME"))
  (shadow 'ReachGoal (find-package "FOOTBALL_GAME")))
(unless (find-package "FOOTBALL_GAME::REACHGOAL")
  (make-package "FOOTBALL_GAME::REACHGOAL"))
(unless (find-package "FOOTBALL_GAME::REACHGOALREQUEST")
  (make-package "FOOTBALL_GAME::REACHGOALREQUEST"))
(unless (find-package "FOOTBALL_GAME::REACHGOALRESPONSE")
  (make-package "FOOTBALL_GAME::REACHGOALRESPONSE"))

(in-package "ROS")

(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))




(defclass football_game::ReachGoalRequest
  :super ros::object
  :slots (_robot_des ))

(defmethod football_game::ReachGoalRequest
  (:init
   (&key
    ((:robot_des __robot_des) (instance nav_msgs::Odometry :init))
    )
   (send-super :init)
   (setq _robot_des __robot_des)
   self)
  (:robot_des
   (&rest __robot_des)
   (if (keywordp (car __robot_des))
       (send* _robot_des __robot_des)
     (progn
       (if __robot_des (setq _robot_des (car __robot_des)))
       _robot_des)))
  (:serialization-length
   ()
   (+
    ;; nav_msgs/Odometry _robot_des
    (send _robot_des :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; nav_msgs/Odometry _robot_des
       (send _robot_des :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; nav_msgs/Odometry _robot_des
     (send _robot_des :deserialize buf ptr-) (incf ptr- (send _robot_des :serialization-length))
   ;;
   self)
  )

(defclass football_game::ReachGoalResponse
  :super ros::object
  :slots (_ack ))

(defmethod football_game::ReachGoalResponse
  (:init
   (&key
    ((:ack __ack) nil)
    )
   (send-super :init)
   (setq _ack __ack)
   self)
  (:ack
   (&optional __ack)
   (if __ack (setq _ack __ack)) _ack)
  (:serialization-length
   ()
   (+
    ;; bool _ack
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _ack
       (if _ack (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _ack
     (setq _ack (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass football_game::ReachGoal
  :super ros::object
  :slots ())

(setf (get football_game::ReachGoal :md5sum-) "959d8712aaa1507c9a1417b801028273")
(setf (get football_game::ReachGoal :datatype-) "football_game/ReachGoal")
(setf (get football_game::ReachGoal :request) football_game::ReachGoalRequest)
(setf (get football_game::ReachGoal :response) football_game::ReachGoalResponse)

(defmethod football_game::ReachGoalRequest
  (:response () (instance football_game::ReachGoalResponse :init)))

(setf (get football_game::ReachGoalRequest :md5sum-) "959d8712aaa1507c9a1417b801028273")
(setf (get football_game::ReachGoalRequest :datatype-) "football_game/ReachGoalRequest")
(setf (get football_game::ReachGoalRequest :definition-)
      "nav_msgs/Odometry robot_des

================================================================================
MSG: nav_msgs/Odometry
# This represents an estimate of a position and velocity in free space.  
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

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
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
---
bool ack



")

(setf (get football_game::ReachGoalResponse :md5sum-) "959d8712aaa1507c9a1417b801028273")
(setf (get football_game::ReachGoalResponse :datatype-) "football_game/ReachGoalResponse")
(setf (get football_game::ReachGoalResponse :definition-)
      "nav_msgs/Odometry robot_des

================================================================================
MSG: nav_msgs/Odometry
# This represents an estimate of a position and velocity in free space.  
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

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
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
---
bool ack



")



(provide :football_game/ReachGoal "959d8712aaa1507c9a1417b801028273")



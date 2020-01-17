; Auto-generated. Do not edit!


(cl:in-package football_game-srv)


;//! \htmlinclude ReachGoal-request.msg.html

(cl:defclass <ReachGoal-request> (roslisp-msg-protocol:ros-message)
  ((robot_des
    :reader robot_des
    :initarg :robot_des
    :type nav_msgs-msg:Odometry
    :initform (cl:make-instance 'nav_msgs-msg:Odometry)))
)

(cl:defclass ReachGoal-request (<ReachGoal-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReachGoal-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReachGoal-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name football_game-srv:<ReachGoal-request> is deprecated: use football_game-srv:ReachGoal-request instead.")))

(cl:ensure-generic-function 'robot_des-val :lambda-list '(m))
(cl:defmethod robot_des-val ((m <ReachGoal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader football_game-srv:robot_des-val is deprecated.  Use football_game-srv:robot_des instead.")
  (robot_des m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReachGoal-request>) ostream)
  "Serializes a message object of type '<ReachGoal-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robot_des) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReachGoal-request>) istream)
  "Deserializes a message object of type '<ReachGoal-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robot_des) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReachGoal-request>)))
  "Returns string type for a service object of type '<ReachGoal-request>"
  "football_game/ReachGoalRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReachGoal-request)))
  "Returns string type for a service object of type 'ReachGoal-request"
  "football_game/ReachGoalRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReachGoal-request>)))
  "Returns md5sum for a message object of type '<ReachGoal-request>"
  "959d8712aaa1507c9a1417b801028273")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReachGoal-request)))
  "Returns md5sum for a message object of type 'ReachGoal-request"
  "959d8712aaa1507c9a1417b801028273")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReachGoal-request>)))
  "Returns full string definition for message of type '<ReachGoal-request>"
  (cl:format cl:nil "nav_msgs/Odometry robot_des~%~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReachGoal-request)))
  "Returns full string definition for message of type 'ReachGoal-request"
  (cl:format cl:nil "nav_msgs/Odometry robot_des~%~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReachGoal-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robot_des))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReachGoal-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ReachGoal-request
    (cl:cons ':robot_des (robot_des msg))
))
;//! \htmlinclude ReachGoal-response.msg.html

(cl:defclass <ReachGoal-response> (roslisp-msg-protocol:ros-message)
  ((ack
    :reader ack
    :initarg :ack
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ReachGoal-response (<ReachGoal-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReachGoal-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReachGoal-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name football_game-srv:<ReachGoal-response> is deprecated: use football_game-srv:ReachGoal-response instead.")))

(cl:ensure-generic-function 'ack-val :lambda-list '(m))
(cl:defmethod ack-val ((m <ReachGoal-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader football_game-srv:ack-val is deprecated.  Use football_game-srv:ack instead.")
  (ack m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReachGoal-response>) ostream)
  "Serializes a message object of type '<ReachGoal-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ack) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReachGoal-response>) istream)
  "Deserializes a message object of type '<ReachGoal-response>"
    (cl:setf (cl:slot-value msg 'ack) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReachGoal-response>)))
  "Returns string type for a service object of type '<ReachGoal-response>"
  "football_game/ReachGoalResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReachGoal-response)))
  "Returns string type for a service object of type 'ReachGoal-response"
  "football_game/ReachGoalResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReachGoal-response>)))
  "Returns md5sum for a message object of type '<ReachGoal-response>"
  "959d8712aaa1507c9a1417b801028273")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReachGoal-response)))
  "Returns md5sum for a message object of type 'ReachGoal-response"
  "959d8712aaa1507c9a1417b801028273")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReachGoal-response>)))
  "Returns full string definition for message of type '<ReachGoal-response>"
  (cl:format cl:nil "bool ack~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReachGoal-response)))
  "Returns full string definition for message of type 'ReachGoal-response"
  (cl:format cl:nil "bool ack~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReachGoal-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReachGoal-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ReachGoal-response
    (cl:cons ':ack (ack msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ReachGoal)))
  'ReachGoal-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ReachGoal)))
  'ReachGoal-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReachGoal)))
  "Returns string type for a service object of type '<ReachGoal>"
  "football_game/ReachGoal")
; Auto-generated. Do not edit!


(cl:in-package f110_occgrid-srv)


;//! \htmlinclude ConvertMap-request.msg.html

(cl:defclass <ConvertMap-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ConvertMap-request (<ConvertMap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConvertMap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConvertMap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name f110_occgrid-srv:<ConvertMap-request> is deprecated: use f110_occgrid-srv:ConvertMap-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConvertMap-request>) ostream)
  "Serializes a message object of type '<ConvertMap-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConvertMap-request>) istream)
  "Deserializes a message object of type '<ConvertMap-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConvertMap-request>)))
  "Returns string type for a service object of type '<ConvertMap-request>"
  "f110_occgrid/ConvertMapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConvertMap-request)))
  "Returns string type for a service object of type 'ConvertMap-request"
  "f110_occgrid/ConvertMapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConvertMap-request>)))
  "Returns md5sum for a message object of type '<ConvertMap-request>"
  "b13d2865c5af2a64e6e30ab1b56e1dd5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConvertMap-request)))
  "Returns md5sum for a message object of type 'ConvertMap-request"
  "b13d2865c5af2a64e6e30ab1b56e1dd5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConvertMap-request>)))
  "Returns full string definition for message of type '<ConvertMap-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConvertMap-request)))
  "Returns full string definition for message of type 'ConvertMap-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConvertMap-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConvertMap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ConvertMap-request
))
;//! \htmlinclude ConvertMap-response.msg.html

(cl:defclass <ConvertMap-response> (roslisp-msg-protocol:ros-message)
  ((image
    :reader image
    :initarg :image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass ConvertMap-response (<ConvertMap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConvertMap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConvertMap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name f110_occgrid-srv:<ConvertMap-response> is deprecated: use f110_occgrid-srv:ConvertMap-response instead.")))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <ConvertMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader f110_occgrid-srv:image-val is deprecated.  Use f110_occgrid-srv:image instead.")
  (image m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConvertMap-response>) ostream)
  "Serializes a message object of type '<ConvertMap-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConvertMap-response>) istream)
  "Deserializes a message object of type '<ConvertMap-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConvertMap-response>)))
  "Returns string type for a service object of type '<ConvertMap-response>"
  "f110_occgrid/ConvertMapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConvertMap-response)))
  "Returns string type for a service object of type 'ConvertMap-response"
  "f110_occgrid/ConvertMapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConvertMap-response>)))
  "Returns md5sum for a message object of type '<ConvertMap-response>"
  "b13d2865c5af2a64e6e30ab1b56e1dd5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConvertMap-response)))
  "Returns md5sum for a message object of type 'ConvertMap-response"
  "b13d2865c5af2a64e6e30ab1b56e1dd5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConvertMap-response>)))
  "Returns full string definition for message of type '<ConvertMap-response>"
  (cl:format cl:nil "sensor_msgs/Image image~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConvertMap-response)))
  "Returns full string definition for message of type 'ConvertMap-response"
  (cl:format cl:nil "sensor_msgs/Image image~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConvertMap-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConvertMap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ConvertMap-response
    (cl:cons ':image (image msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ConvertMap)))
  'ConvertMap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ConvertMap)))
  'ConvertMap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConvertMap)))
  "Returns string type for a service object of type '<ConvertMap>"
  "f110_occgrid/ConvertMap")
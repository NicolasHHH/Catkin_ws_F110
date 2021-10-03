;; Auto-generated. Do not edit!


(when (boundp 'f110_occgrid::ConvertMap)
  (if (not (find-package "F110_OCCGRID"))
    (make-package "F110_OCCGRID"))
  (shadow 'ConvertMap (find-package "F110_OCCGRID")))
(unless (find-package "F110_OCCGRID::CONVERTMAP")
  (make-package "F110_OCCGRID::CONVERTMAP"))
(unless (find-package "F110_OCCGRID::CONVERTMAPREQUEST")
  (make-package "F110_OCCGRID::CONVERTMAPREQUEST"))
(unless (find-package "F110_OCCGRID::CONVERTMAPRESPONSE")
  (make-package "F110_OCCGRID::CONVERTMAPRESPONSE"))

(in-package "ROS")



(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass f110_occgrid::ConvertMapRequest
  :super ros::object
  :slots ())

(defmethod f110_occgrid::ConvertMapRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass f110_occgrid::ConvertMapResponse
  :super ros::object
  :slots (_image ))

(defmethod f110_occgrid::ConvertMapResponse
  (:init
   (&key
    ((:image __image) (instance sensor_msgs::Image :init))
    )
   (send-super :init)
   (setq _image __image)
   self)
  (:image
   (&rest __image)
   (if (keywordp (car __image))
       (send* _image __image)
     (progn
       (if __image (setq _image (car __image)))
       _image)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/Image _image
    (send _image :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/Image _image
       (send _image :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/Image _image
     (send _image :deserialize buf ptr-) (incf ptr- (send _image :serialization-length))
   ;;
   self)
  )

(defclass f110_occgrid::ConvertMap
  :super ros::object
  :slots ())

(setf (get f110_occgrid::ConvertMap :md5sum-) "b13d2865c5af2a64e6e30ab1b56e1dd5")
(setf (get f110_occgrid::ConvertMap :datatype-) "f110_occgrid/ConvertMap")
(setf (get f110_occgrid::ConvertMap :request) f110_occgrid::ConvertMapRequest)
(setf (get f110_occgrid::ConvertMap :response) f110_occgrid::ConvertMapResponse)

(defmethod f110_occgrid::ConvertMapRequest
  (:response () (instance f110_occgrid::ConvertMapResponse :init)))

(setf (get f110_occgrid::ConvertMapRequest :md5sum-) "b13d2865c5af2a64e6e30ab1b56e1dd5")
(setf (get f110_occgrid::ConvertMapRequest :datatype-) "f110_occgrid/ConvertMapRequest")
(setf (get f110_occgrid::ConvertMapRequest :definition-)
      "---
sensor_msgs/Image image

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

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

(setf (get f110_occgrid::ConvertMapResponse :md5sum-) "b13d2865c5af2a64e6e30ab1b56e1dd5")
(setf (get f110_occgrid::ConvertMapResponse :datatype-) "f110_occgrid/ConvertMapResponse")
(setf (get f110_occgrid::ConvertMapResponse :definition-)
      "---
sensor_msgs/Image image

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

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



(provide :f110_occgrid/ConvertMap "b13d2865c5af2a64e6e30ab1b56e1dd5")



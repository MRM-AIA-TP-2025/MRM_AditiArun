;; Auto-generated. Do not edit!


(when (boundp 'rover::ToggleIMURepresentation)
  (if (not (find-package "ROVER"))
    (make-package "ROVER"))
  (shadow 'ToggleIMURepresentation (find-package "ROVER")))
(unless (find-package "ROVER::TOGGLEIMUREPRESENTATION")
  (make-package "ROVER::TOGGLEIMUREPRESENTATION"))
(unless (find-package "ROVER::TOGGLEIMUREPRESENTATIONREQUEST")
  (make-package "ROVER::TOGGLEIMUREPRESENTATIONREQUEST"))
(unless (find-package "ROVER::TOGGLEIMUREPRESENTATIONRESPONSE")
  (make-package "ROVER::TOGGLEIMUREPRESENTATIONRESPONSE"))

(in-package "ROS")





(defclass rover::ToggleIMURepresentationRequest
  :super ros::object
  :slots (_x _y _z _w ))

(defmethod rover::ToggleIMURepresentationRequest
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:w __w) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _w (float __w))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:w
   (&optional __w)
   (if __w (setq _w __w)) _w)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ;; float64 _w
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _w
       (sys::poke _w (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _w
     (setq _w (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass rover::ToggleIMURepresentationResponse
  :super ros::object
  :slots (_roll _pitch _yaw ))

(defmethod rover::ToggleIMURepresentationResponse
  (:init
   (&key
    ((:roll __roll) 0.0)
    ((:pitch __pitch) 0.0)
    ((:yaw __yaw) 0.0)
    )
   (send-super :init)
   (setq _roll (float __roll))
   (setq _pitch (float __pitch))
   (setq _yaw (float __yaw))
   self)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:serialization-length
   ()
   (+
    ;; float64 _roll
    8
    ;; float64 _pitch
    8
    ;; float64 _yaw
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _roll
     (setq _roll (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pitch
     (setq _pitch (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass rover::ToggleIMURepresentation
  :super ros::object
  :slots ())

(setf (get rover::ToggleIMURepresentation :md5sum-) "52512739d54d35724c9fe339cf727a07")
(setf (get rover::ToggleIMURepresentation :datatype-) "rover/ToggleIMURepresentation")
(setf (get rover::ToggleIMURepresentation :request) rover::ToggleIMURepresentationRequest)
(setf (get rover::ToggleIMURepresentation :response) rover::ToggleIMURepresentationResponse)

(defmethod rover::ToggleIMURepresentationRequest
  (:response () (instance rover::ToggleIMURepresentationResponse :init)))

(setf (get rover::ToggleIMURepresentationRequest :md5sum-) "52512739d54d35724c9fe339cf727a07")
(setf (get rover::ToggleIMURepresentationRequest :datatype-) "rover/ToggleIMURepresentationRequest")
(setf (get rover::ToggleIMURepresentationRequest :definition-)
      "float64 x
float64 y
float64 z
float64 w
---
float64 roll
float64 pitch
float64 yaw


")

(setf (get rover::ToggleIMURepresentationResponse :md5sum-) "52512739d54d35724c9fe339cf727a07")
(setf (get rover::ToggleIMURepresentationResponse :datatype-) "rover/ToggleIMURepresentationResponse")
(setf (get rover::ToggleIMURepresentationResponse :definition-)
      "float64 x
float64 y
float64 z
float64 w
---
float64 roll
float64 pitch
float64 yaw


")



(provide :rover/ToggleIMURepresentation "52512739d54d35724c9fe339cf727a07")



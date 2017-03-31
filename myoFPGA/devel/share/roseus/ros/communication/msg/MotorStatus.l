;; Auto-generated. Do not edit!


(when (boundp 'communication::MotorStatus)
  (if (not (find-package "COMMUNICATION"))
    (make-package "COMMUNICATION"))
  (shadow 'MotorStatus (find-package "COMMUNICATION")))
(unless (find-package "COMMUNICATION::MOTORSTATUS")
  (make-package "COMMUNICATION::MOTORSTATUS"))

(in-package "ROS")
;;//! \htmlinclude MotorStatus.msg.html


(defclass communication::MotorStatus
  :super ros::object
  :slots (_pwmRef _position _velocity _displacement _current ))

(defmethod communication::MotorStatus
  (:init
   (&key
    ((:pwmRef __pwmRef) (make-array 0 :initial-element 0 :element-type :integer))
    ((:position __position) (make-array 0 :initial-element 0 :element-type :integer))
    ((:velocity __velocity) (make-array 0 :initial-element 0 :element-type :integer))
    ((:displacement __displacement) (make-array 0 :initial-element 0 :element-type :integer))
    ((:current __current) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _pwmRef __pwmRef)
   (setq _position __position)
   (setq _velocity __velocity)
   (setq _displacement __displacement)
   (setq _current __current)
   self)
  (:pwmRef
   (&optional __pwmRef)
   (if __pwmRef (setq _pwmRef __pwmRef)) _pwmRef)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:displacement
   (&optional __displacement)
   (if __displacement (setq _displacement __displacement)) _displacement)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:serialization-length
   ()
   (+
    ;; int32[] _pwmRef
    (* 4    (length _pwmRef)) 4
    ;; int32[] _position
    (* 4    (length _position)) 4
    ;; int16[] _velocity
    (* 2    (length _velocity)) 4
    ;; int16[] _displacement
    (* 2    (length _displacement)) 4
    ;; int16[] _current
    (* 2    (length _current)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _pwmRef
     (write-long (length _pwmRef) s)
     (dotimes (i (length _pwmRef))
       (write-long (elt _pwmRef i) s)
       )
     ;; int32[] _position
     (write-long (length _position) s)
     (dotimes (i (length _position))
       (write-long (elt _position i) s)
       )
     ;; int16[] _velocity
     (write-long (length _velocity) s)
     (dotimes (i (length _velocity))
       (write-word (elt _velocity i) s)
       )
     ;; int16[] _displacement
     (write-long (length _displacement) s)
     (dotimes (i (length _displacement))
       (write-word (elt _displacement i) s)
       )
     ;; int16[] _current
     (write-long (length _current) s)
     (dotimes (i (length _current))
       (write-word (elt _current i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _pwmRef
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pwmRef (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _pwmRef i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _position (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _position i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int16[] _velocity
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocity (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _velocity i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; int16[] _displacement
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _displacement (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _displacement i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; int16[] _current
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _current (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _current i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;;
   self)
  )

(setf (get communication::MotorStatus :md5sum-) "8e3cb2e63688fce1513f561de7a5ce50")
(setf (get communication::MotorStatus :datatype-) "communication/MotorStatus")
(setf (get communication::MotorStatus :definition-)
      "int32[] pwmRef
int32[] position
int16[] velocity
int16[] displacement
int16[] current
")



(provide :communication/MotorStatus "8e3cb2e63688fce1513f561de7a5ce50")



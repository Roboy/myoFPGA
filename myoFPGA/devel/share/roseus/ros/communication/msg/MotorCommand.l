;; Auto-generated. Do not edit!


(when (boundp 'communication::MotorCommand)
  (if (not (find-package "COMMUNICATION"))
    (make-package "COMMUNICATION"))
  (shadow 'MotorCommand (find-package "COMMUNICATION")))
(unless (find-package "COMMUNICATION::MOTORCOMMAND")
  (make-package "COMMUNICATION::MOTORCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude MotorCommand.msg.html


(defclass communication::MotorCommand
  :super ros::object
  :slots (_motors _setPoints ))

(defmethod communication::MotorCommand
  (:init
   (&key
    ((:motors __motors) (make-array 0 :initial-element 0 :element-type :char))
    ((:setPoints __setPoints) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _motors __motors)
   (setq _setPoints __setPoints)
   self)
  (:motors
   (&optional __motors)
   (if __motors (setq _motors __motors)) _motors)
  (:setPoints
   (&optional __setPoints)
   (if __setPoints (setq _setPoints __setPoints)) _setPoints)
  (:serialization-length
   ()
   (+
    ;; uint8[] _motors
    (* 1    (length _motors)) 4
    ;; int32[] _setPoints
    (* 4    (length _setPoints)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8[] _motors
     (write-long (length _motors) s)
     (princ _motors s)
     ;; int32[] _setPoints
     (write-long (length _setPoints) s)
     (dotimes (i (length _setPoints))
       (write-long (elt _setPoints i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8[] _motors
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _motors (make-array n :element-type :char))
     (replace _motors buf :start2 ptr-) (incf ptr- n))
   ;; int32[] _setPoints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _setPoints (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _setPoints i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get communication::MotorCommand :md5sum-) "7da1664b5fc700acbf0efa5165c4eb21")
(setf (get communication::MotorCommand :datatype-) "communication/MotorCommand")
(setf (get communication::MotorCommand :definition-)
      "uint8[] motors
int32[] setPoints
")



(provide :communication/MotorCommand "7da1664b5fc700acbf0efa5165c4eb21")



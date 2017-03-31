;; Auto-generated. Do not edit!


(when (boundp 'communication::MotorConfig)
  (if (not (find-package "COMMUNICATION"))
    (make-package "COMMUNICATION"))
  (shadow 'MotorConfig (find-package "COMMUNICATION")))
(unless (find-package "COMMUNICATION::MOTORCONFIG")
  (make-package "COMMUNICATION::MOTORCONFIG"))

(in-package "ROS")
;;//! \htmlinclude MotorConfig.msg.html


(defclass communication::MotorConfig
  :super ros::object
  :slots (_control_mode _outputPosMax _outputNegMax _spPosMax _spNegMax _Kp _Ki _Kd _forwardGain _deadBand _IntegralPosMax _IntegralNegMax ))

(defmethod communication::MotorConfig
  (:init
   (&key
    ((:control_mode __control_mode) (make-array 0 :initial-element 0 :element-type :char))
    ((:outputPosMax __outputPosMax) (make-array 0 :initial-element 0 :element-type :integer))
    ((:outputNegMax __outputNegMax) (make-array 0 :initial-element 0 :element-type :integer))
    ((:spPosMax __spPosMax) (make-array 0 :initial-element 0 :element-type :integer))
    ((:spNegMax __spNegMax) (make-array 0 :initial-element 0 :element-type :integer))
    ((:Kp __Kp) (make-array 0 :initial-element 0 :element-type :integer))
    ((:Ki __Ki) (make-array 0 :initial-element 0 :element-type :integer))
    ((:Kd __Kd) (make-array 0 :initial-element 0 :element-type :integer))
    ((:forwardGain __forwardGain) (make-array 0 :initial-element 0 :element-type :integer))
    ((:deadBand __deadBand) (make-array 0 :initial-element 0 :element-type :integer))
    ((:IntegralPosMax __IntegralPosMax) (make-array 0 :initial-element 0 :element-type :integer))
    ((:IntegralNegMax __IntegralNegMax) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _control_mode __control_mode)
   (setq _outputPosMax __outputPosMax)
   (setq _outputNegMax __outputNegMax)
   (setq _spPosMax __spPosMax)
   (setq _spNegMax __spNegMax)
   (setq _Kp __Kp)
   (setq _Ki __Ki)
   (setq _Kd __Kd)
   (setq _forwardGain __forwardGain)
   (setq _deadBand __deadBand)
   (setq _IntegralPosMax __IntegralPosMax)
   (setq _IntegralNegMax __IntegralNegMax)
   self)
  (:control_mode
   (&optional __control_mode)
   (if __control_mode (setq _control_mode __control_mode)) _control_mode)
  (:outputPosMax
   (&optional __outputPosMax)
   (if __outputPosMax (setq _outputPosMax __outputPosMax)) _outputPosMax)
  (:outputNegMax
   (&optional __outputNegMax)
   (if __outputNegMax (setq _outputNegMax __outputNegMax)) _outputNegMax)
  (:spPosMax
   (&optional __spPosMax)
   (if __spPosMax (setq _spPosMax __spPosMax)) _spPosMax)
  (:spNegMax
   (&optional __spNegMax)
   (if __spNegMax (setq _spNegMax __spNegMax)) _spNegMax)
  (:Kp
   (&optional __Kp)
   (if __Kp (setq _Kp __Kp)) _Kp)
  (:Ki
   (&optional __Ki)
   (if __Ki (setq _Ki __Ki)) _Ki)
  (:Kd
   (&optional __Kd)
   (if __Kd (setq _Kd __Kd)) _Kd)
  (:forwardGain
   (&optional __forwardGain)
   (if __forwardGain (setq _forwardGain __forwardGain)) _forwardGain)
  (:deadBand
   (&optional __deadBand)
   (if __deadBand (setq _deadBand __deadBand)) _deadBand)
  (:IntegralPosMax
   (&optional __IntegralPosMax)
   (if __IntegralPosMax (setq _IntegralPosMax __IntegralPosMax)) _IntegralPosMax)
  (:IntegralNegMax
   (&optional __IntegralNegMax)
   (if __IntegralNegMax (setq _IntegralNegMax __IntegralNegMax)) _IntegralNegMax)
  (:serialization-length
   ()
   (+
    ;; uint8[] _control_mode
    (* 1    (length _control_mode)) 4
    ;; int32[] _outputPosMax
    (* 4    (length _outputPosMax)) 4
    ;; int32[] _outputNegMax
    (* 4    (length _outputNegMax)) 4
    ;; int32[] _spPosMax
    (* 4    (length _spPosMax)) 4
    ;; int32[] _spNegMax
    (* 4    (length _spNegMax)) 4
    ;; uint16[] _Kp
    (* 2    (length _Kp)) 4
    ;; uint16[] _Ki
    (* 2    (length _Ki)) 4
    ;; uint16[] _Kd
    (* 2    (length _Kd)) 4
    ;; uint16[] _forwardGain
    (* 2    (length _forwardGain)) 4
    ;; uint16[] _deadBand
    (* 2    (length _deadBand)) 4
    ;; int32[] _IntegralPosMax
    (* 4    (length _IntegralPosMax)) 4
    ;; int32[] _IntegralNegMax
    (* 4    (length _IntegralNegMax)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8[] _control_mode
     (write-long (length _control_mode) s)
     (princ _control_mode s)
     ;; int32[] _outputPosMax
     (write-long (length _outputPosMax) s)
     (dotimes (i (length _outputPosMax))
       (write-long (elt _outputPosMax i) s)
       )
     ;; int32[] _outputNegMax
     (write-long (length _outputNegMax) s)
     (dotimes (i (length _outputNegMax))
       (write-long (elt _outputNegMax i) s)
       )
     ;; int32[] _spPosMax
     (write-long (length _spPosMax) s)
     (dotimes (i (length _spPosMax))
       (write-long (elt _spPosMax i) s)
       )
     ;; int32[] _spNegMax
     (write-long (length _spNegMax) s)
     (dotimes (i (length _spNegMax))
       (write-long (elt _spNegMax i) s)
       )
     ;; uint16[] _Kp
     (write-long (length _Kp) s)
     (dotimes (i (length _Kp))
       (write-word (elt _Kp i) s)
       )
     ;; uint16[] _Ki
     (write-long (length _Ki) s)
     (dotimes (i (length _Ki))
       (write-word (elt _Ki i) s)
       )
     ;; uint16[] _Kd
     (write-long (length _Kd) s)
     (dotimes (i (length _Kd))
       (write-word (elt _Kd i) s)
       )
     ;; uint16[] _forwardGain
     (write-long (length _forwardGain) s)
     (dotimes (i (length _forwardGain))
       (write-word (elt _forwardGain i) s)
       )
     ;; uint16[] _deadBand
     (write-long (length _deadBand) s)
     (dotimes (i (length _deadBand))
       (write-word (elt _deadBand i) s)
       )
     ;; int32[] _IntegralPosMax
     (write-long (length _IntegralPosMax) s)
     (dotimes (i (length _IntegralPosMax))
       (write-long (elt _IntegralPosMax i) s)
       )
     ;; int32[] _IntegralNegMax
     (write-long (length _IntegralNegMax) s)
     (dotimes (i (length _IntegralNegMax))
       (write-long (elt _IntegralNegMax i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8[] _control_mode
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _control_mode (make-array n :element-type :char))
     (replace _control_mode buf :start2 ptr-) (incf ptr- n))
   ;; int32[] _outputPosMax
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _outputPosMax (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _outputPosMax i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _outputNegMax
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _outputNegMax (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _outputNegMax i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _spPosMax
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _spPosMax (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _spPosMax i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _spNegMax
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _spNegMax (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _spNegMax i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; uint16[] _Kp
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _Kp (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _Kp i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint16[] _Ki
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _Ki (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _Ki i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint16[] _Kd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _Kd (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _Kd i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint16[] _forwardGain
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _forwardGain (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _forwardGain i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; uint16[] _deadBand
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _deadBand (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _deadBand i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; int32[] _IntegralPosMax
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _IntegralPosMax (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _IntegralPosMax i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _IntegralNegMax
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _IntegralNegMax (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _IntegralNegMax i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get communication::MotorConfig :md5sum-) "051c9c5b4a5a7dd6e3d99abc0b0331c6")
(setf (get communication::MotorConfig :datatype-) "communication/MotorConfig")
(setf (get communication::MotorConfig :definition-)
      "uint8[] control_mode
int32[] outputPosMax
int32[] outputNegMax
int32[] spPosMax
int32[] spNegMax
uint16[] Kp
uint16[] Ki
uint16[] Kd
uint16[] forwardGain
uint16[] deadBand
int32[] IntegralPosMax
int32[] IntegralNegMax
")



(provide :communication/MotorConfig "051c9c5b4a5a7dd6e3d99abc0b0331c6")



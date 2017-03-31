
(cl:in-package :asdf)

(defsystem "communication-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MotorConfig" :depends-on ("_package_MotorConfig"))
    (:file "_package_MotorConfig" :depends-on ("_package"))
    (:file "MotorStatus" :depends-on ("_package_MotorStatus"))
    (:file "_package_MotorStatus" :depends-on ("_package"))
  ))
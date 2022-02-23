
(cl:in-package :asdf)

(defsystem "ds4_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "DS4" :depends-on ("_package_DS4"))
    (:file "_package_DS4" :depends-on ("_package"))
  ))
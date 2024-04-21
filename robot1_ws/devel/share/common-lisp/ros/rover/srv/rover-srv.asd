
(cl:in-package :asdf)

(defsystem "rover-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ToggleIMURepresentation" :depends-on ("_package_ToggleIMURepresentation"))
    (:file "_package_ToggleIMURepresentation" :depends-on ("_package"))
  ))
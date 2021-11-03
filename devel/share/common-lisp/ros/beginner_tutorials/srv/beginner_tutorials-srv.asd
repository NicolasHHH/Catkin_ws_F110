
(cl:in-package :asdf)

(defsystem "beginner_tutorials-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SommerDeuxEnts" :depends-on ("_package_SommerDeuxEnts"))
    (:file "_package_SommerDeuxEnts" :depends-on ("_package"))
  ))
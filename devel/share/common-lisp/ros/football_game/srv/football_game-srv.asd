
(cl:in-package :asdf)

(defsystem "football_game-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :nav_msgs-msg
)
  :components ((:file "_package")
    (:file "ReachGoal" :depends-on ("_package_ReachGoal"))
    (:file "_package_ReachGoal" :depends-on ("_package"))
  ))
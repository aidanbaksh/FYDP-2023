
(cl:in-package :asdf)

(defsystem "lidar_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "BoxInfo" :depends-on ("_package_BoxInfo"))
    (:file "_package_BoxInfo" :depends-on ("_package"))
    (:file "Curb" :depends-on ("_package_Curb"))
    (:file "_package_Curb" :depends-on ("_package"))
    (:file "Lane" :depends-on ("_package_Lane"))
    (:file "_package_Lane" :depends-on ("_package"))
    (:file "PointStamped" :depends-on ("_package_PointStamped"))
    (:file "_package_PointStamped" :depends-on ("_package"))
    (:file "SocketBox" :depends-on ("_package_SocketBox"))
    (:file "_package_SocketBox" :depends-on ("_package"))
    (:file "box" :depends-on ("_package_box"))
    (:file "_package_box" :depends-on ("_package"))
  ))
#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "jsk_perception")

(ros::roseus "tmp-server")

(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

(defvar cur-x 0)
(defvar cur-y 0)
(defvar cur-z 0)
(defvar cur-prob 0)
(defvar msg-tmp (instance (class (send (instance jsk_perception::DetectionAction :init) :action_result)) :init))

(defun demo-cb(msg)
  (let ((tm))
    (setq cur-x (+ cur-x 1))
    (setq cur-y (+ cur-y 1))
    (setq cur-z (+ cur-z 1))
    (setq cur-prob (+ cur-prob 1))
    (ros::ros-info "x:~A y:~A z:~A :prob:~A" cur-x cur-y cur-z cur-prob)
    )
  )

(defun server-cb (server goal)
  (let ((order (send goal :goal :order))
        (sequence (list 1 0))
        msg
        )
;    (setq msg (instance (class (send (instance jsk_perception::Detection2Action :init) :action_result)) :init))
    (setq msg (send server :result))
    (send* msg :result (list :x (list cur-x)))
    (send* msg :result (list :y (list cur-y)))
    (send* msg :result (list :z (list cur-z)))
    (send* msg :result (list :prob (list cur-prob)))
    (send server :set-succeeded msg)
    (ros::ros-info "hoge")
    ))

(defun demo()
  (setq s (instance ros::simple-action-server :init
                    "fibonacci" jsk_perception::Detection2Action
                    :execute-cb 'server-cb));;#'とするとエラーする
  (ros::subscribe "/tmp" std_msgs::Float64 #'demo-cb)
  (ros::rate 200)
  (do-until-key
   (send s :worker)
   (ros::spin-once)
   (ros::sleep))
  )
#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "jsk_perception")

(ros::roseus "tmp-client")
(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

(ros::service-call "~set_logger_level"
                   (instance roscpp::SetLoggerLevelRequest
                             :init :logger "ros" :level "DEBUG"))
;;;
;;;
(defun fibonacci (n)
  (if (< n 2)
      n
    (+ (fibonacci (- n 1)) (fibonacci (- n 2)))))

(defun fibonacci-client (&optional (order 10))
  (let (goal)
    (if (not (boundp '*c*))
        (setq *c* (instance ros::simple-action-client :init
                            "fibonacci" jsk_perception::Detection2Action)))
    (warning-message 2 ";; ~A wait-for-server~%" (unix::getpid))
    (send *c* :wait-for-server)
    (setq goal (instance jsk_perception::Detection2ActionGoal :init))
    (send goal :goal :order order)
    (send *c* :send-goal goal)

    (warning-message 2 ";; ~A wait-for-result~%" (unix::getpid))
    (send *c* :wait-for-result)
    (warning-message 2 ";; ~A result -> ~A~A~A~A~%" (unix::getpid) (send (send *c* :get-result) :x) (send (send *c* :get-result) :y) (send (send *c* :get-result) :z) (send (send *c* :get-result) :prob))
    (send (send *c* :get-result) :x))
  )

(defun demo (&optional (order 1))
  (let (goal)
    (if (not (boundp '*c*))
        (setq *c* (instance ros::simple-action-client :init
                            "fibonacci" jsk_perception::Detection2Action)))
    (warning-message 2 ";; ~A wait-for-server~%" (unix::getpid))
    (send *c* :wait-for-server)
    (setq goal (instance jsk_perception::Detection2ActionGoal :init))
    (send goal :goal :order order)
    (send *c* :send-goal goal)
    (warning-message 2 ";; ~A wait-for-result~%" (unix::getpid))
    (send *c* :wait-for-result)
    (warning-message 2 ";; ~A result -> ~A~A~A~A~%" (unix::getpid) (send (send *c* :get-result) :x) (send (send *c* :get-result) :y) (send (send *c* :get-result) :z) (send (send *c* :get-result) :prob))
    )
  )
(demo)


;; (let ((count 0) ans fib)  ;; for test-fibonaci.launch
;;   (setq ans (fibonacci 10))
;;   (do-until-key
;;    (warning-message 4 ";; test-fibonacci ~A/~A~%" count 1000)
;;    (incf count)
;;    (when (> count 1000)
;;      (exit 0))
;;    (setq fib (fibonacci-client 10))
;;    (if (> (length fib) 0)
;;        (assert  (= (elt fib (1- (length fib))) ans) "(fib 10) -> ~A" fib))
;;    (exit 0)
;;    )

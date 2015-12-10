

(load "package://jsk_perception/euslisp/detection_interface.l")


(defun fr ()
  (setq cds (check-detection-srv
             :type "fridge"
             :detection-topic "/kinect_head/rgb/ObjectDetection"
             )))

(defun can ()
  (setq cds (check-detection-srv
             :type "georgia"
             :detection-topic "/kinect_head_c2/depth_registered/ObjectDetection")))


(ros::advertise "detectcanflag" std_msgs::Bool 10)
(ros::advertise "detectfridgeflag" std_msgs::Bool 10)

(defun st ()
  (let (msg)
    (setq msg (instance std_msgs::Bool))
    (send msg :data t) ;; end detecting fridge door
    (ros::publish "detectfridgeflag" msg)
    ))

(defun end ()
  (let (msg)
    (setq msg (instance std_msgs::Bool))
    (send msg :data nil) ;; end detecting fridge door
    (ros::publish "detectfridgeflag" msg)
    ))

#|

rosnode list
rosnode info /fridge_marker

|#


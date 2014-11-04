(ros::load-ros-manifest "roseus")

(ros::roseus "tmp-pub")

(defun demo()
  nil
  (ros::advertise "/tmp" std_msgs::Float64 1)
  (ros::rate 1)
  (do-until-key
   (ros::spin-once)
   (ros::publish "/tmp" (instance std_msgs::Float64 :init :data 1))
   (ros::ros-info "publish float64")
   (ros::sleep))
  )


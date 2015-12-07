
(ros::roseus "pub")
(ros::load-ros-manifest "posedetection_msgs")
(ros::roseus-add-msgs "posedetection_msgs")

(ros::rate 1)

(ros::advertise "/kinect_head_c2/depth_registered/ObjectDetection" posedetection_msgs::ObjectDetection)
(setq objdet (instance posedetection_msgs::ObjectDetection :init))
(setq obj (instance posedetection_msgs::Object6DPose :init
                    :pose
                     (instance geometry_msgs::Pose :init
                               :position
                               (instance geometry_msgs::Point :init
                                         :x 1
                                         :y 2
                                         :z 3)
                               :orientation
                               (instance geometry_msgs::Quaternion :init
                                         :x 0
                                         :y 0
                                         :z 0
                                         :w 1))))

(send objdet :objects (list obj))
;(send objdet :header :frame_id "/head_mount_kinect_rgb_optical_frame")
(send objdet :header :frame_id "/base_link")

(defun cb (msg)
  (send objdet :header (send msg :header))
  (ros::publish "/kinect_head_c2/depth_registered/ObjectDetection" objdet)
  )

(ros::subscribe "/kinect_head_c2/rgb/camera_info" sensor_msgs::CameraInfo #'cb)

(ros::spin)


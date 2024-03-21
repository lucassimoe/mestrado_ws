## run docker

docker run --privileged --network="host" -v /dev:/dev -it lucas/mestrado:3.0 bash

roslaunch realsense2_camera rs_camera.launch align_depth:=true

roslaunch rtabmap_launch rtabmap.launch rtabmap_args:="--delete_db_on_start" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false
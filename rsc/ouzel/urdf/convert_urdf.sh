rosrun xacro xacro -o model.urdf ouzel_base.xacro mav_name:=ouzel namespace:=ouzel enable_ground_truth:=false enable_logging:=false add_manipulator:=false add_rgbd_sensor:=false add_tether:=false
gz sdf -p model.urdf > model.sdf
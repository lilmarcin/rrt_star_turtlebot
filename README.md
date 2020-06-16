# rrt_star_turtlebot

roslaunch rrt_miapr colloseum.launch

rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map odom 100

rosrun rrt_miapr rrt_star.py

rortun rrt_miapr controller.py

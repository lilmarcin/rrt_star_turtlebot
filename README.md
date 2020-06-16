# rrt_star_turtlebot

roslaunch rrt_star_turtlebot colloseum.launch

rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map odom 100

rosrun rrt_star_turtlebot rrt_star.py

rosrun rrt_star_turtlebot controller.py

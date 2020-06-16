# rrt_star_turtlebot

![rrt*](https://cdn.discordapp.com/attachments/465526373369511956/722519046179782656/unknown.png)

Implementacja algorytmu RRT* do planowania ruchu pojazdu w układzie differential drive (robot turtlebot) na mapie zajętości.

Komendy do wywołania:
Uruchomienie środowiska gazebo+rviz

**roslaunch rrt_star_turtlebot colloseum.launch**

Przekształcenie pomiędzy układem mapy a odom

**rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map odom 100**

Uruchomienie algorytmu rrt*

**rosrun rrt_star_turtlebot rrt_star.py**

Uruchomienie kontrolera turtlebot

**rosrun rrt_star_turtlebot controller.py**

#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, PoseStamped
from math import atan2
import numpy as np


x = 0.0
y = 0.0
theta = 0.0
cnt = 1
ok = 1
distance = 999999
pathX = []
pathY = []
angle = []
cnt_kat = 0

with open('/home/marcinxd/catkin_ws/src/rrt_miapr/src/path.txt', "r") as file1:
    f_list = [float(i) for line in file1 for i in line.split(' ') if i.strip()]
    T = f_list[0]
print(T)

for i in range(len(f_list)):
    if i%2 == 0:
        pathX.append(f_list[i])
    else:
        pathY.append(f_list[i])


def newOdom(msg):
    global x
    global y
    global theta
    global robotPath
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    # create path
    robotPath.header.frame_id = "odom"
    robotPath.header.stamp = rospy.Time.now()
    pose = PoseStamped()
    pose.header = msg.header
    pose.pose = msg.pose.pose
    robotPath.poses.append(pose)

rospy.init_node("speed_controller")

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
robot_path_pub = rospy.Publisher("/robot_path", Path, queue_size=10)
r = rospy.Rate(4)
speed = Twist() #Speed
robotPath = Path() #Robot path
goal = Point() #Desired location


while not rospy.is_shutdown():
    #xd = odom.pose.pose.position.x

    goal.x = pathX[cnt]
    goal.y = pathY[cnt]
    print('target x:', pathX[cnt], ' robot x: ', x)
    print('target y:', pathY[cnt], ' robot y: ', y)

    inc_x = goal.x - x
    inc_y = goal.y - y

    #print('y',inc_y)

    angle_to_goal = atan2(inc_y, inc_x)
    #print('x', angle_to_goal)
    #print('x', x)
    #print('y', y)

    angle.append(angle_to_goal)
    zero_crossings = np.where(np.diff(np.sign(angle)))[0]
    print('zero crossings: ', len(zero_crossings))
    if len(zero_crossings)>3:
        ok = 0


    distance = np.sqrt((goal.x - x)**2 + (goal.y - y)**2)
    print('distance', distance)
    print(' ')
    #if abs(angle_to_goal - theta) > 0.1 & & angle_to_goal - theta < 0:

    if angle_to_goal - theta >= 0.1 and ok == 1:
        speed.linear.x = 0.0
        speed.angular.z = 0.3
    elif angle_to_goal - theta <= -0.1 and ok == 1:
        speed.linear.x = 0.0
        speed.angular.z = -0.3
    elif abs(angle_to_goal - theta) >= 0.1 and ok == 0:
        print('obrot w kierunku nastepnego punktu')
        speed.linear.x = 0.0
        speed.angular.z = -0.5
    elif abs(angle_to_goal - theta) <= 0.1 and ok == 0:
        del angle[:]
        ok = 1
        zero_crossings = 0
    elif distance < 0.1:
        ok = 1
        cnt += 1
        #print('xddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd')
        speed.linear.x = 0.0
        speed.angular.z = 0.0

        del angle[:]
    else:
        speed.linear.x = 0.3
        speed.angular.z = 0.0

    # publish path
    robot_path_pub.publish(robotPath)
    pub.publish(speed)
    r.sleep()

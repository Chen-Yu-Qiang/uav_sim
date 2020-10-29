#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import String
from tello_driver.msg import TelloStatus
import rospy
import time
import numpy as np

kp = 1
ki = 0
kd = 0

x = 0
y = 0
z = 0
t = 0
already_takeoff = 0

ref_x = 0
ref_y = 0
ref_z = 0
ref_t = 0
now_x = 0
now_y = 0
now_z = 0
now_t = 0
bat = 100
theta = 180

leader_cmd_msg=Twist()

def cb_updata_ref(data):
    global ref_x, ref_y, ref_z, ref_t
    ref_x = data.linear.x
    ref_y = data.linear.y
    ref_z = data.linear.z
    ref_t = data.angular.z
    if ref_t<0:
        ref_t=ref_t+2*np.pi
    if ref_t>2*np.pi:
        ref_t=ref_t-2*np.pi


def cb_updata_err(data):
    global x, y, z, t
    x = data.linear.x
    y = data.linear.y
    z = data.linear.z
    t = data.angular.z


def cb_takeoff(data):
    global already_takeoff
    time.sleep(5)
    already_takeoff = 1


def cb_land(data):
    global already_takeoff, pub_cmd
    already_takeoff = 0
    cmdmsg = Twist()
    cmdmsg.linear.x = 0
    cmdmsg.linear.y = 0
    cmdmsg.linear.z = 0
    cmdmsg.angular.z = 0
    pub_cmd.publish(cmdmsg)


def cb_updata_now(data):
    global now_x, now_y, now_z, now_t,theta
    now_x = data.pose.position.x
    now_y = data.pose.position.y
    now_z = data.pose.position.z
    now_t = np.arctan2(data.pose.orientation.z,data.pose.orientation.w)*2
    if now_t<0:
        now_t=now_t+np.pi*2
    theta=now_t

def cb_stste(data):
    global bat
    bat = data.battery_percentage


def cb_updata_theta(data):
    global theta
    theta = np.arctan2(data.pose.pose.orientation.z,data.pose.pose.orientation.w)*2
    if theta<0:
        theta=theta+np.pi*2
def cb_LeaderVel(data):
    global leader_cmd_msg
    leader_cmd_msg.linear.x=data.linear.x
    leader_cmd_msg.linear.y=data.linear.y
    leader_cmd_msg.linear.z=data.linear.z
    leader_cmd_msg.angular.z=data.angular.z

rospy.init_node('controlTello', anonymous=True)
pub_cmd = rospy.Publisher('tello/cmd_vel', Twist, queue_size=1)
pub_cmd_world = rospy.Publisher('cmd_world', Twist, queue_size=1)
pub_str = rospy.Publisher('str_log', String, queue_size=10)
rospy.Subscriber('err', Twist, cb_updata_err)
rospy.Subscriber('tello/takeoff', Empty, cb_takeoff)
rospy.Subscriber('tello/land', Empty, cb_land)
rospy.Subscriber('genRef', Twist, cb_updata_ref)
rospy.Subscriber('mixIMUMarker', PoseStamped, cb_updata_now)
rospy.Subscriber("tello/status", TelloStatus, cb_stste)
rospy.Subscriber("/drone1/tello/cmd_vel", Twist, cb_LeaderVel)
#rospy.Subscriber("tello/odom", Odometry, cb_updata_theta)
rate = rospy.Rate(20)
err_i_x = 0
err_i_y = 0
err_i_z = 0
err_i_t = 0
err_x = 0
err_y = 0
err_z = 0
err_t = 0
cmd_x=0
cmd_y=0
cmd_z=0
cmd_t=0
cmdmsg = Twist()
while not rospy.is_shutdown():
    if already_takeoff:
        kp = 2
        ki = 0
        kd = 0
        err_d_x = err_x
        err_x = x
        err_i_x = err_i_x+x
        err_d_x = x-err_d_x
        cmd_x = kp*err_x+ki*err_i_x+kd*err_d_x

        err_d_y = err_y
        err_y = y
        err_i_y = err_i_y+y
        err_d_y = y-err_d_y
        cmd_y = kp*err_y+ki*err_i_y+kd*err_d_y

        kp = 1
        ki = 0
        kd = 0
        err_d_z = err_z
        err_z = z
        err_i_z = err_i_z+z
        err_d_z = z-err_d_z
        cmd_z = kp*err_z+ki*err_i_z+kd*err_d_z

        kp = 1
        ki = 0
        kd = 0
        err_d_t = err_t
        err_t = t
        err_i_t = err_i_t+t
        err_d_t = t-err_d_t
        cmd_t = kp*err_t+ki*err_i_t+kd*err_d_t

    # q0=cos(theta/2)
    # q123=(x,y,z)sin(theta/2)
    #now_t
    #   Z{w}
    #   |
    # X o--Y
    #
    #      Z{c0}
    #      |
    #  X --o Y
    #
    #    Z{c90}
    #    |
    #  X o-- Y
    #
    #    Z{c180}
    #    |
    #  Y x-- X
    #

    if already_takeoff:
        if cmdmsg.linear.x == cmd_y and cmdmsg.linear.y == -cmd_x and cmdmsg.linear.z == cmd_z and 0:
            pass
        else:
            # cmdmsg.linear.x=cmd_y
            # cmdmsg.linear.y=-cmd_x
            # cmdmsg.linear.z=cmd_z
            # pub_cmd.publish(cmdmsg)
            #   {c}  s  -c   0   {w}
            #     p=[c   s   0] *  p
            #        0   0   1
            #TODO 
            dddd=np.sqrt((ref_x+x)*(ref_x+x)+(ref_y+y)*(ref_y+y))
            cmdmsg.linear.x = np.sin(theta)*cmd_x-np.cos(theta)*cmd_y+leader_cmd_msg.linear.x+leader_cmd_msg.angular.z*dddd*0.9486
            cmdmsg.linear.y = np.cos(theta)*cmd_x+np.sin(theta)*cmd_y+leader_cmd_msg.linear.y-leader_cmd_msg.angular.z*dddd*0.3162
            cmdmsg.linear.z = cmd_z
            cmdmsg.angular.z = cmd_t+leader_cmd_msg.angular.z
            #cmdmsg.linear.x = 1
            #cmdmsg.linear.y = 0
            #cmdmsg.linear.z = 0
            #cmdmsg.angular.z = 0
            pub_cmd.publish(cmdmsg)
    else:
        cmdmsg.linear.x = 0
        cmdmsg.linear.y = 0
        cmdmsg.linear.z = 0
    #print("%d,%f,%f,%f,%f" % (bat, now_x, now_y, now_z, now_t*180/np.pi))

    print("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f" % (
        time.time(), ref_x, ref_y, ref_z, ref_t*180/np.pi,
        now_x, now_y, now_z, now_t*180/np.pi, x, y, z,
        t*180/np.pi, cmd_x, cmd_y, cmd_z, cmd_t))
    pub_str.publish("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f" % (
        time.time(), ref_x, ref_y, ref_z, ref_t,
        now_x, now_y, now_z, now_t, x, y, z,
        t, cmd_x, cmd_y, cmd_z, cmd_t))
    world_cmd_msg=Twist()
    world_cmd_msg.linear.x=cmd_x
    world_cmd_msg.linear.y=cmd_y
    world_cmd_msg.linear.z=cmd_z
    world_cmd_msg.angular.z=cmd_t
    pub_cmd_world.publish(world_cmd_msg)

    rate.sleep()

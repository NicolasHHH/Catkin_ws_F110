#!/usr/bin/env python

"""
ref:
https://github.com/ROBOTIS-GIT/turtlebot3/blob/master/turtlebot3_teleop/nodes/turtlebot3_teleop_key
https://stackoverflow.com/questions/24072790/detect-key-press-in-python
"""

import rospy
import std_msgs
from pynput.keyboard import Key, Listener,KeyCode
from ackermann_msgs.msg import AckermannDriveStamped

MAX_LIN_VEL = 3.0  # max vel lin 3 m/s
MAX_ANG_VEL = 0.52 # max vel ang 30 degrees

LIN_VEL_STEP_SIZE = 0.3
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your TurtleBot3!
---------------------------
Moving around by keyboard arrows:
            up
   left    down    right

up/down : increase/decrease linear velocity
left/right : increase/decrease angular velocity

shift : force stop

CTRL-C to quit
"""

def vels(linear_vel, angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (linear_vel,angular_vel)

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

class Agent(object):
    def __init__(self):
        self.drive_pub_ego = rospy.Publisher('ego_id/drive', AckermannDriveStamped, queue_size=1)
        self.drive_pub_opp = rospy.Publisher('opp_id/drive', AckermannDriveStamped, queue_size=1)
        self.ego_linear_vel   = 0.0
        self.ego_angular_vel  = 0.0
        self.opp_linear_vel   = 0.0
        self.opp_angular_vel  = 0.0

    def publish_cmd_ego(self, event=None):
        drive = AckermannDriveStamped()
        drive.drive.speed = self.ego_linear_vel
        drive.drive.steering_angle = self.ego_angular_vel
        #print("publish drive speed : ",drive.drive.speed)
        self.drive_pub_ego.publish(drive)
        self.ego_angular_vel /= 1.1 

    def publish_cmd_opp(self, event=None):
        drive = AckermannDriveStamped()
        drive.drive.speed = self.opp_linear_vel
        drive.drive.steering_angle = self.opp_angular_vel
        #print("publish drive speed : ",drive.drive.speed)
        self.drive_pub_opp.publish(drive)
        self.opp_angular_vel  /= 1.1

    def on_press(self, key):
        if (key==Key.up):
            self.ego_linear_vel = constrain(self.ego_linear_vel + LIN_VEL_STEP_SIZE, -MAX_LIN_VEL, MAX_LIN_VEL)
            print(vels(self.ego_linear_vel, self.ego_angular_vel))
        elif (key==Key.down):
            self.ego_linear_vel = constrain(self.ego_linear_vel - LIN_VEL_STEP_SIZE, -MAX_LIN_VEL, MAX_LIN_VEL)
            print(vels(self.ego_linear_vel, self.ego_angular_vel))
        elif (key==Key.left):
            self.ego_angular_vel = 0.51
        elif (key==Key.right):
            self.ego_angular_vel = -0.51
        elif (key==Key.backspace):
            self.ego_linear_vel   = 0.0
            self.ego_angular_vel  = 0.0
            print(vels(self.ego_linear_vel, self.ego_angular_vel))

        elif (key==KeyCode.from_char('y')):
            self.opp_linear_vel = constrain(self.opp_linear_vel + LIN_VEL_STEP_SIZE, -MAX_LIN_VEL, MAX_LIN_VEL)
            print(vels(self.opp_linear_vel, self.opp_angular_vel))
        elif (key==KeyCode.from_char('h')):
            self.opp_linear_vel = constrain(self.opp_linear_vel - LIN_VEL_STEP_SIZE, -MAX_LIN_VEL, MAX_LIN_VEL)
            print(vels(self.opp_linear_vel, self.opp_angular_vel))
        elif (key==KeyCode.from_char('g')):
            self.opp_angular_vel = 0.51
        elif (key==KeyCode.from_char('j')):
            self.opp_angular_vel = -0.51
        elif (key==Key.shift):
            self.opp_linear_vel   = 0.0
            self.opp_angular_vel  = 0.0
            print(vels(self.opp_linear_vel, self.opp_angular_vel))

    def on_press_action(self,key):
        self.on_press(key)

    def keep_listenning(self, event=None):
        # Collect events until released
        with Listener(on_press=self.on_press) as listener:
            listener.join()

if __name__ == '__main__':
    rospy.init_node('keyboard_node')
    print(msg)

    agent = Agent()
    rospy.Timer(rospy.Duration(1.0/100.0), agent.keep_listenning)
    rospy.Timer(rospy.Duration(1.0/100.0), agent.publish_cmd_ego)
    rospy.Timer(rospy.Duration(1.0/100.0), agent.publish_cmd_opp)
    rospy.spin()

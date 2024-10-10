import os, sys
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import roslaunch
import time
import rospy
from sensor_msgs.msg import Joy
from key_mapper import KEYMAP
from panda import Panda
import subprocess
import utils


def move_gripper(x=0.0, y=0.0, z=0.0):
    wpose = panda.end_effector_pose
    wpose.position.x += x
    wpose.position.y += y
    wpose.position.z += z
    panda.move_through_waypoints([wpose])
    
def move_home():
    home_config = [-2.2748631408944138, 0.974557520662193, 0.6354566556779961,
                   -2.3196569263832387, 1.2403121058379996, 2.060963524952071,
                   -2.2097670971912553]
    panda.move_to_configuration(home_config)

def joystick_callback(data):

    if data.buttons[KEYMAP['x']]:
        panda.fully_close_gripper()
    elif data.buttons[KEYMAP['o']]:
        panda.fully_open_gripper()
    #elif data.buttons[KEYMAP['up-arrow']]:
    #    move_gripper(x=0.03)
    #elif data.buttons[KEYMAP['down-arrow']]:
    #    move_gripper(x=-0.03)
    #elif data.buttons[KEYMAP['left-arrow']]:
    #    move_gripper(y=0.03)
    #elif data.buttons[KEYMAP['right-arrow']]:
    #    move_gripper(y=-0.03)
    elif data.buttons[KEYMAP['l1']]:
        move_gripper(z=0.01)
    elif data.buttons[KEYMAP['r1']]:
        move_gripper(z=-0.01)
    elif data.buttons[KEYMAP['pskey']]:
        move_home()

if __name__ == '__main__':
    rospy.init_node('panda_ps_control')

    panda = Panda()
    rospy.Subscriber('joy', Joy, joystick_callback, queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

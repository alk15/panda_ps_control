import time
import rospy
from sensor_msgs.msg import Joy
from key_mapper import KEYMAP
from panda import Panda
import subprocess
import signal
import utils

def move_gripper(linear_x=0.0, linear_y=0.0, angular_z=0.0):
    wpose = panda.end_effector_pose
    if linear_x != 0.0:
        wpose.position.x += linear_x * 0.1
    if linear_y != 0.0:
        wpose.position.y += linear_y * 0.1
    panda.move_through_waypoints([wpose])

def launch_particle_filtering():
    if still_killing: # Killing previous process isn't finished yet; wait.
        return

    commands = [
        'roslaunch PBPF code_visual.launch'
        'rosrun image_view image_view image:=/dope/rgb_points_my_wireframe'
    ]

    for command in commands:
        process = subprocess.Popen(command, shell=True)
        subprocesses.append(process)

    time.sleep(1)
    manage_windows()

def manage_windows():
    commands = [
        'wmctrl -i -r $(wmctrl -l | grep "/dope/rgb_points_my_wireframe" | awk "{print $1}") -e "0,0,0,960,-1"'
        'wmctrl -i -r $(wmctrl -l | grep "DOPE" | awk "{print $1}") -e "0,960,0,960,540"'
        'wmctrl -i -r $(wmctrl -l | grep "PBPF" | awk "{print $1}") -e "0,960,540,960,540"'
    ]

    for command in commands:
        subprocess.Popen(command, shell=True)

def kill_particle_filtering():
    global still_killing
    still_killing = True
    for process in subprocesses:
        process.send_signal(signal.SIGINT)
        process.wait()
    still_killing = False

def joystick_callback(data):
    if DEBUG:
        utils.print_keys_pressed(data.buttons)

    if data.buttons[KEYMAP['start']]:
        launch_particle_filtering()
    elif data.buttons[KEYMAP['select']]:
        kill_particle_filtering()
    elif data.buttons[KEYMAP['x']]:
        panda.fully_open_gripper()
    elif data.buttons[KEYMAP['o']]:
        panda.fully_close_gripper()
    else:
        linear_x = data.axes[0]
        linear_y = data.axes[1]
        move_gripper(linear_x, linear_y)


if __name__ == '__main__':
    still_killing = False
    rospy.init_node('panda-ps3-controller')
    subprocesses = []
    panda = Panda()
    rospy.Subscriber('joy', Joy, joystick_callback)
    rospy.spin()

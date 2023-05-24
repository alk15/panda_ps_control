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

def launch_particle_filtering():
    global particle_filter_running

    if particle_filter_running: # Killing previous process isn't finished yet; wait.
        print('Particle filtering already running')
        return
    
    particle_filter_running = True
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    path = os.path.expanduser('~/catkin_ws/src/PBPF')
    launch = roslaunch.parent.ROSLaunchParent(uuid, [f'{path}/launch/code_visuial.launch'])
    launch.start()
    time.sleep(2)
    manage_windows()

def manage_windows():
    commands = [
        'wmctrl -i -r $(wmctrl -l | grep "/dope/rgb_points_my_wireframe" | awk "{print $1}") -e "0,0,0,960,1080"',
        'wmctrl -i -r $(wmctrl -l | grep "Bullet" | sed -n 1p | awk "{print $1}") -e "0,960,0,960,500"',
        'wmctrl -i -r $(wmctrl -l | grep "Bullet" | sed -n 2p | awk "{print $1}") -e "0,960,510,960,500"',
    ]

    for command in commands:
        subprocess.Popen(command, shell=True)

def kill_particle_filtering():
    global particle_filter_running
    if particle_filter_running:
        print('Killing particle filter')
        os.system('pkill -9 Visualisation')
        os.system('pkill -9 Physics_Based_')
        os.system('pkill -9 image_view')
        particle_filter_running = False

def joystick_callback(data):
    global start_particle_filter, particle_filter_running

    if data.buttons[KEYMAP['start']]:
        if not particle_filter_running:
            start_particle_filter = True
    elif data.buttons[KEYMAP['select']]:
        kill_particle_filtering()
    elif data.buttons[KEYMAP['x']]:
        panda.fully_close_gripper()
    elif data.buttons[KEYMAP['o']]:
        panda.fully_open_gripper()
    elif data.buttons[KEYMAP['up-arrow']]:
        move_gripper(x=0.03)
    elif data.buttons[KEYMAP['down-arrow']]:
        move_gripper(x=-0.03)
    elif data.buttons[KEYMAP['left-arrow']]:
        move_gripper(y=0.03)
    elif data.buttons[KEYMAP['right-arrow']]:
        move_gripper(y=-0.03)
    elif data.buttons[KEYMAP['l1']]:
        move_gripper(z=0.01)
    elif data.buttons[KEYMAP['r1']]:
        move_gripper(z=-0.01)
    elif data.buttons[KEYMAP['pskey']]:
        move_home()

if __name__ == '__main__':
    rospy.init_node('panda-ps3-controller')
    particle_filter_running = False
    start_particle_filter = False

    panda = Panda()
    rospy.Subscriber('joy', Joy, joystick_callback, queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if start_particle_filter:
            launch_particle_filtering()
            start_particle_filter = False
        rate.sleep()

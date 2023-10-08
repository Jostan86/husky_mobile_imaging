#!/usr/bin/env python3
import rospy
from functools import partial
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty, Trigger

button_states = None
record_state = False


def button_callback(msg):
    """Callback for the joystick buttons"""

    global button_states

    # Check if the message is old
    if (rospy.Time.now() - msg.header.stamp).to_sec() > 1.0:
        rospy.logwarn('Received an old Joy message, disregarding...')
        return

    # Do the button callbacks that were pressed
    for i in callbacks:
        new_state = msg.buttons[i]
        if new_state and not button_states[i]:
            button_states[i] = True
            callbacks[i]()
        elif not new_state and button_states[i]:
            button_states[i] = False

def default_callback(i):
    rospy.loginfo('Pressed Button {}'.format(i))

def do_record():
    """start/stop recording"""
    global record_state
    if record_state:
        stop_record_srv()
        rospy.loginfo('Stopping recording')
        record_state = False
    else:
        record_srv()
        rospy.loginfo('Starting recording')
        record_state = True

def run_scan(indicator, record=True):
    """Run a scan, record data if record=True"""
    srv_name = '/imaging_platform/scan_actuator_{}'.format(indicator)
    rospy.wait_for_service(srv_name, timeout=5.0)
    scan_srv = rospy.ServiceProxy(srv_name, Trigger)
    if record:
        record_srv()
    try:
        scan_srv()
    finally:
        if record:
            stop_record_srv()

def home_act(indicator):
    """Tell actuator to home"""
    srv_name = '/imaging_platform/home_actuator_{}'.format(indicator)
    rospy.wait_for_service(srv_name, timeout=5.0)
    home_srv = rospy.ServiceProxy(srv_name, Trigger)
    home_srv()

if __name__ == '__main__':

    rospy.init_node('joy_listener')

    # LOGITECH: 0=A, 1=B, 2=X, 3=Y, 4/5 are left/right triggers, 6=Back, 7=Start, 9=Left Stick, 10=Right Stick

    # Initialize the button states
    msg = rospy.wait_for_message('/joy_teleop/joy', Joy)
    buttons = len(msg.buttons)
    button_states = [False] * buttons

    # Define the callbacks
    callbacks = {
        9: partial(run_scan, 'V'),
        # 10: partial(run_scan, 'H'),
        7: partial(run_scan, 'V', False),
        # 6: partial(run_scan, 'H', False),
        3: partial(home_act, 'V'),
        # 1: partial(home_act, 'H'),
        2: do_record,
    }  # Key = button index, value = callback

    # Wait for the record service and then connect to it
    rospy.wait_for_service('/imaging_platform/record_data')
    record_srv = rospy.ServiceProxy('/imaging_platform/record_data', Empty)
    stop_record_srv = rospy.ServiceProxy('/imaging_platform/stop_record_data', Empty)

    # Start the subscriber to the buttons
    rospy.Subscriber('/joy_teleop/joy', Joy, button_callback, queue_size=1)

    rospy.spin()


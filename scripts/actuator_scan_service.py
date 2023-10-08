#!/usr/bin/env python3
import time
import rospy
from std_srvs.srv import Trigger
from functools import partial
from husky_mobile_imaging.msg import ActuatorStatus, ActuatorPosition, ActuatorCmd

class ActuatorState:
    def __init__(self, device_name, move_dist, units_per_mm=1000, friendly_id=None):
        """ Actuator state object, used to keep track of the state of the actuator"""
        # Name of actuator, v_actuator or h_actuator
        self.device_name = device_name
        # Distance to move during scan
        self.move_dist = move_dist
        # Units per mm, 1000 since the actuator is in microns
        self.units_per_mm = units_per_mm
        # Friendly ID, short name for the actuator
        self.friendly_id = friendly_id or device_name

        # State variables
        self.active = False
        self.current_position = 0

    def update_active(self, val):
        self.active = val

    def update_position(self, position):
        self.current_position = position / self.units_per_mm

class ScanService:
    def __init__(self):
        """Services for the actuator"""

        # Move speed in mm/s
        self.move_speed = 50

        # Define the devices
        self.devices = {
            'v_actuator': ActuatorState('v_actuator', 840, friendly_id='V'),      # Vertical
            'h_actuator': ActuatorState('v_actuator', 340, friendly_id='H'),      # Horizontal
        }

        self.last_status_received = rospy.Time()

        # Define the publisher and subscribers
        self.pub = rospy.Publisher('/imaging_platform/actuator_control', ActuatorCmd, queue_size=10)
        self.status_sub = rospy.Subscriber('/imaging_platform/actuator_status', ActuatorStatus, self.status_callback)
        self.position_sub = rospy.Subscriber('/imaging_platform/actuator_positions', ActuatorPosition, self.position_callback)

        # Define the services
        for device_id in self.devices:
            status = self.devices[device_id]
            rospy.Service(f'/imaging_platform/scan_actuator_{status.friendly_id}', Trigger,
                          partial(self.handle_scan, device_id))
            rospy.Service(f'/imaging_platform/home_actuator_{status.friendly_id}', Trigger,
                          partial(self.handle_homing, device_id))

    def position_callback(self, msg):
        """Callback for the positions of the actuators"""
        self.devices['v_actuator'].update_position(msg.v_act_pos)
        self.devices['h_actuator'].update_position(msg.h_act_pos)

    def request_update(self, block=True):
        """Request an update from the actuator"""
        last_msg_stamp = self.last_status_received
        self.publish_cmd(['status_update', '', 0, 0])
        if block:
            while self.last_status_received == last_msg_stamp:
                rospy.sleep(0.1)

    def publish_cmd(self, command):
        """Publish a command to the actuator"""
        cmd_msg = ActuatorCmd()
        cmd_msg.cmd_type.data = command[0]
        cmd_msg.actuator_name.data = command[1]
        cmd_msg.position = command[2]
        cmd_msg.velocity = command[3]
        self.pub.publish(cmd_msg)

    def await_device_threshold(self, device_name, dist, more_than=True):
        """Wait for the device to reach a certain position"""
        while True:
            if more_than:
                cond = self.devices[device_name].current_position >= dist - 1e-6
            else:
                cond = self.devices[device_name].current_position <= dist + 1e-6
            if cond:
                return
            rospy.sleep(0.1)

    def handle_scan(self, device_name, _):
        """Handle the camera scan, first moves the actuator to the end, then back to the start"""
        self.request_update(block=True)
        device = self.devices[device_name]

        if device.active:
            # Forward
            self.publish_cmd(['move', device_name, device.move_dist, self.move_speed])
            self.await_device_threshold(device_name, device.move_dist, more_than=True)

            # Backwards
            reverse_thres = 0
            self.publish_cmd(['move', device_name, reverse_thres, self.move_speed])
            self.await_device_threshold(device_name, reverse_thres, more_than=False)

            return True, "Scan complete"

        else:
            return False, 'Device is not active'

    def handle_homing(self, device_name, _):
        """Tells the actuator to enable itself"""
        self.request_update(block=True)
        device = self.devices[device_name]

        if device.active:
            return True, 'Device already Enabled'
        else:
            self.publish_cmd(['enable', device_name, 0, 0])

            while not self.devices[device_name].active:
                rospy.sleep(0.1)

            return True, 'Device Enabled'


    def status_callback(self, status_msg):
        """Callback for the status of the actuator"""
        self.devices['v_actuator'].update_active(bool(status_msg.v_act_homed))
        self.devices['h_actuator'].update_active(bool(status_msg.h_act_homed))

        self.last_status_received = rospy.Time.now()


if __name__ == "__main__":
    rospy.init_node('actuator_scan_server')
    scan_service = ScanService()
    rospy.spin()
#!/usr/bin/env python3
import os.path

import rospy
from std_srvs.srv import Empty
import subprocess
import shlex
import shutil

class DataRecorder:

    def __init__(self, to_record, save_loc, ns=''):
        """Setup the data recorder

        Args:
            to_record (list): List of topics to record
            save_loc (str): Location to save the data
            """

        self.to_record = to_record
        self.save_loc = save_loc
        self.active_process = None

        prefix = ''
        if ns:
            prefix = ns.strip('/') + '/'

        rospy.Service(prefix + '/imaging_platform/record_data', Empty, self.start_recording)
        rospy.Service(prefix + '/imaging_platform/stop_record_data', Empty, self.stop_recording)

    def stop_recording(self, *_, **__):
        """Stop recording data"""
        if self.active_process is not None:
            self.active_process.terminate()
            rospy.loginfo('Recording stopped')
        self.active_process = None
        return []

    def start_recording(self, *_, **__):
        """Start recording data"""
        if self.active_process is not None:
            self.active_process.terminate()

        file_path = get_new_file_path(save_loc)
        msg = "rosbag record -O {} --split --duration 60 ".format(file_path) + ' '.join(self.to_record)
        print(msg)

        args = shlex.split(msg)
        self.active_process = subprocess.Popen(args, stderr=subprocess.PIPE, shell=False)
        return []

def get_new_file_path(folder, file_format='{}.bag'):
    import time
    cur_time = int(time.time())
    return os.path.join(folder, file_format.format(cur_time))

def shutdown():
    """Shutdown the node"""

    recorder.stop_recording()
    if hot_swap_loc:
        import time
        time.sleep(1.5)
        finished_bags = [x for x in os.listdir(save_loc) if x.endswith('.bag') and 'active' not in x]
        for to_move in finished_bags:
            shutil.move(os.path.join(save_loc, to_move), os.path.join(hot_swap_loc, to_move))

if __name__ == '__main__':
    rospy.init_node('record_data')

    # Topics to record
    topics = [
                '/camera/color/camera_info',
                '/camera/depth/camera_info',
                '/camera/color/image_raw',
                '/camera/depth/image_rect_raw',
                '/camera/aligned_depth_to_color/camera_info',
                '/camera/aligned_depth_to_color/image_raw',
                '/front/scan',
                '/odometry/filtered',
                '/imu/data',
                '/tf',
                '/imaging_platform/actuator_positions',
              ]

    # Save location, this must exist for the node to work
    save_loc = os.path.join(os.path.expanduser('~'), 'data', 'tree_scanning')
    if not os.path.exists(save_loc):
        rospy.logwarn('No save path found at {}'.format(save_loc))
        raise Exception()

    # Setup the recorder
    recorder = DataRecorder(topics, save_loc)
    rospy.on_shutdown(recorder.stop_recording)

    # Check for a hot swap drive and set it up if it exists
    mounts = os.listdir('/media/jostan')
    hot_swap_loc = None
    for mount in mounts:
        proposed_save_loc = os.path.join('/media/jostan', mount, 'bags')
        if os.path.exists(proposed_save_loc):
            hot_swap_loc = proposed_save_loc
            rospy.loginfo('Hot swap mount found: {}'.format(proposed_save_loc))
            break
    if hot_swap_loc is None:
        rospy.logwarn('No hot swap drive found!')
        rospy.spin()
    else:
        while not rospy.is_shutdown():
            finished_bags = [x for x in os.listdir(save_loc) if x.endswith('.bag') and 'active' not in x]
            if finished_bags:
                to_move = finished_bags[0]
                shutil.move(os.path.join(save_loc, to_move), os.path.join(hot_swap_loc, to_move))
            rospy.sleep(1.0)

    rospy.spin()
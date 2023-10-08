import os
import cv2
from collections import defaultdict
import rosbag
from cv_bridge import CvBridge

def proc_video(bag_files, cam_topic, output_path):

    out = None
    bridge = CvBridge()
    try:
        for bag_file in bag_files:
            bag = rosbag.Bag(bag_file)
            info = bag.get_type_and_topic_info()
            if cam_topic not in info[1]:
                throttled_topic = '/throttled' + cam_topic
                if throttled_topic in info[1]:
                    print('Found throttled topic')
                    cam_topic = throttled_topic
            for topic, msg, t in bag.read_messages(topics=[cam_topic]):
                if topic != cam_topic:
                    continue

                img = bridge.imgmsg_to_cv2(msg, "bgr8")
                h, w, _ = img.shape
                if out is None:
                    frames = bag.get_type_and_topic_info()[1][cam_topic][3]
                    out = cv2.VideoWriter(output_path, cv2.VideoWriter_fourcc(*'MP4V'), frames, (w,h))

                out.write(img)

    finally:
        if out is not None:
            out.release()

if __name__ == '__main__':
    bags_loc = '/media/main/LaCie/bags'
    output_loc = os.path.join(os.path.expanduser('~'), 'Videos', 'outputs')
    files = sorted([x for x in os.listdir(bags_loc) if x.endswith('.bag')])
    sorted_files = defaultdict(list)
    for file in files:
        comps = file.split('_')
        ser = comps[0]
        num = int(comps[1].replace('.bag', ''))
        sorted_files[ser].append(num)

    for ser in sorted_files:
        output_path = os.path.join(output_loc, '{}.avi'.format(ser))
        if os.path.exists(output_path):
            print('Video for series {} found, skipping...'.format(ser))
            continue
        print('Working on series {}'.format(ser))
        files = [os.path.join(bags_loc, '{}_{}.bag'.format(ser, i)) for i in sorted(sorted_files[ser])]

        proc_video(files, '/camera/color/image_raw', output_path)
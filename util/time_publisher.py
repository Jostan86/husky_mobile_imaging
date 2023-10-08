#!/usr/bin/env python3

import rospy
from std_msgs.msg import Time


def time_publisher():
    # Initialize the node
    rospy.init_node('time_publisher_node', anonymous=True)

    # Create a publisher
    pub = rospy.Publisher('current_time', Time, queue_size=10)

    # Set the rate
    rate = rospy.Rate(20)  # 20 Hz

    while not rospy.is_shutdown():
        # Get the current time
        current_time = rospy.Time.now()

        # Publish the time
        pub.publish(current_time)

        # Sleep for the remaining time to maintain 20 Hz rate
        rate.sleep()


if __name__ == '__main__':
    try:
        time_publisher()
    except rospy.ROSInterruptException:
        pass

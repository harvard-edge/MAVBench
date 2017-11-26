#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import tf

def main():
    # Parameters
    localization_method = rospy.get_param('/slam_loss/localization_method',
            'ground_truth')
    timeout = float(rospy.get_param('/slam_loss/timeout', '0'))

    # Publishers
    pub = rospy.Publisher('slam_lost', Bool, queue_size=10, latch=True)

    # Initialize node
    rospy.init_node('slam_loss')

    listener = tf.TransformListener()
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        slam_lost = False

        try:
            now = rospy.Time.now()
            listener.waitForTransform('/world', '/'+localization_method,
                    now, rospy.Duration(timeout))
            listener.lookupTransform('/world', '/'+localization_method, now)
        except (tf.Exception):
            slam_lost = True

        pub.publish(slam_lost)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


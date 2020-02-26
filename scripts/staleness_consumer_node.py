#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

rospy.init_node('staleness_consumer')

samples = []
required_samples = rospy.get_param("~required_samples", default=10)


def image_callback(msg):
    staleness = (rospy.Time.now() - msg.header.stamp).to_sec()
    samples.append(staleness)
    if len(samples) >= required_samples:
        calcStatistics()
        rospy.signal_shutdown('enough samples gathered')
    rospy.loginfo("Processing message with staleness {}".format(staleness))
    rospy.sleep(1.0)


def calcStatistics():
    average = sum(samples) / len(samples)
    sq_sum = sum((s - average) ** 2 for s in samples)
    stdev = (sq_sum / len(samples)) ** 0.5
    rospy.loginfo("Average staleness: {}; standard deviation: {} (based on {} samples)".format(
        average, stdev, len(samples)
    ))


img_sub = rospy.Subscriber('/staleness_producer/image', Image, image_callback, queue_size=1, buff_size=640*480*100)
rospy.spin()

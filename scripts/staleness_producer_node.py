#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

rospy.init_node("staleness_producer")
img_pub = rospy.Publisher("~image", Image, queue_size=1)


def timer_callback(event):
    image = Image()
    image.data = b'\0' * 640 * 480 * 3
    image.width = 640
    image.height = 480
    image.step = 640 * 3
    image.encoding = 'rgb8'
    image.header.stamp = rospy.Time.now()
    img_pub.publish(image)


timer = rospy.Timer(rospy.Duration(0.01), timer_callback)
rospy.spin()

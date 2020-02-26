#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

ros::Publisher producer_publisher;

static void timerCallback(const ros::TimerEvent& event)
{
    sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
    image_msg->data.resize(640 * 480 * 3);
    image_msg->width = 640;
    image_msg->height = 480;
    image_msg->step = 640 * 3;
    image_msg->encoding = sensor_msgs::image_encodings::RGB8;
    image_msg->header.stamp = ros::Time::now();

    producer_publisher.publish(image_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "staleness_producer");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    producer_publisher = nh_priv.advertise<sensor_msgs::Image>("image", 1);
    ros::Timer producer_timer = nh.createTimer(ros::Duration(0.01), &timerCallback);

    ros::spin();

    return 0;
}

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

namespace staleness
{
class Producer : public nodelet::Nodelet
{
private:
    ros::Timer producer_timer;
    ros::Publisher producer_publisher;

    void timerCallback(const ros::TimerEvent& event)
    {
        //cv_bridge::CvImage cv_image;
        //cv_image.image = cv::Mat(cv::Size(640, 480), CV_8UC3);
        sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
        image_msg->data.resize(640 * 480 * 3);
        image_msg->width = 640;
        image_msg->height = 480;
        image_msg->step = 640 * 3;
        image_msg->encoding = sensor_msgs::image_encodings::RGB8;
        image_msg->header.stamp = ros::Time::now();

        producer_publisher.publish(image_msg);
    }
public:
    virtual void onInit()
    {
        ros::NodeHandle& nh = getNodeHandle();
        ros::NodeHandle& nh_priv = getPrivateNodeHandle();
        producer_publisher = nh_priv.advertise<sensor_msgs::Image>("image", 1);
        producer_timer = nh.createTimer(ros::Duration(0.01), &Producer::timerCallback, this);
        
    }
};

}

PLUGINLIB_EXPORT_CLASS(staleness::Producer, nodelet::Nodelet);

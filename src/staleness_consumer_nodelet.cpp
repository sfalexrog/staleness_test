#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/Image.h>
#include <thread>
#include <chrono>

namespace staleness
{
class Consumer : public nodelet::Nodelet
{
private:
    ros::Subscriber img_sub;

    int num_samples, required_samples;
    std::vector<double> samples;

    void image_callback(const sensor_msgs::ImageConstPtr& img_msg)
    {
        auto now = ros::Time::now();
        auto staleness = (now - img_msg->header.stamp).toSec();
        samples.push_back(staleness);
        if (samples.size() >= required_samples)
        {
            calcStatistics();
            std::exit(0);
        }
        NODELET_INFO_STREAM("Processing message with staleness " << staleness);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    void calcStatistics()
    {
        double average = 0.0;
        for (const auto& sample : samples)
        {
            average += sample;
        }
        average /= samples.size();
        double sq_sum = 0.0;
        for (const auto& sample: samples)
        {
            sq_sum += (sample - average) * (sample - average);
        }
        double stdev = std::sqrt(sq_sum / samples.size());

        NODELET_INFO_STREAM("Average staleness: " << average << "; standard deviation: " << stdev << " (based on " << samples.size() << " samples)");
    }

public:
    virtual void onInit()
    {
        auto& nh = getNodeHandle();
        auto& nh_priv = getPrivateNodeHandle();

        img_sub = nh.subscribe<sensor_msgs::Image>("/staleness_producer/image", 1, &Consumer::image_callback, this);
        required_samples = nh_priv.param<int>("required_samples", 10);
        samples.reserve(required_samples);
    }
};

}

PLUGINLIB_EXPORT_CLASS(staleness::Consumer, nodelet::Nodelet);

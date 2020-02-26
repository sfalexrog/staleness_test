#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <thread>
#include <chrono>

ros::Subscriber img_sub;
int num_samples, required_samples;
std::vector<double> samples;

static void calcStatistics();

static void image_callback(const sensor_msgs::ImageConstPtr& img_msg)
{
    auto now = ros::Time::now();
    auto staleness = (now - img_msg->header.stamp).toSec();
    samples.push_back(staleness);
    if (samples.size() >= required_samples)
    {
        calcStatistics();
        std::exit(0);
    }
    ROS_INFO_STREAM("Processing message with staleness " << staleness);
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

static void calcStatistics()
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

    ROS_INFO_STREAM("Average staleness: " << average << "; standard deviation: " << stdev << " (based on " << samples.size() << " samples)");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "staleness_consumer");
    ros::NodeHandle nh, nh_priv("~");
    img_sub = nh.subscribe<sensor_msgs::Image>("/staleness_producer/image", 1, &image_callback);
    required_samples = nh_priv.param<int>("required_samples", 10);
    samples.reserve(required_samples);

    ros::spin();
    return 0;
}

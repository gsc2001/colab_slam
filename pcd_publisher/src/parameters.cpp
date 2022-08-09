#include <ros/ros.h>

#include "parameters.h"

std::string TF_PREFIX, DEPTH_IMAGE_TOPIC, RGB_IMAGE_TOPIC, config_file;
int pcd_stride;

template <typename T>
T readParam(ros::NodeHandle &nh, std::string name)
{
    T ans;
    if (nh.getParam(name, ans)) ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        nh.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &nh) {
    config_file = readParam<std::string>(nh, "config_file");
    TF_PREFIX = readParam<std::string>(nh, "tf_prefix");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);

    if(!fsSettings.isOpened()) {
        std::cerr << "ERROR: Wrong path to settings: " << config_file << std::endl;
    }

    fsSettings["rgb_image_topic"] >> RGB_IMAGE_TOPIC;
    fsSettings["depth_image_topic"] >> DEPTH_IMAGE_TOPIC;
    fsSettings["pcd_stride"] >> pcd_stride;

}

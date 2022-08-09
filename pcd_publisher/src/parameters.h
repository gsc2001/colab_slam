#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <fstream>

extern std::string TF_PREFIX;
extern std::string DEPTH_IMAGE_TOPIC;
extern std::string RGB_IMAGE_TOPIC;
extern std::string config_file;
extern int pcd_stride;

void readParameters(ros::NodeHandle &n);


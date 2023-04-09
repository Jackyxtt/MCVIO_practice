#ifndef MCVIOFRONTEND_H
#define MCVIOFRONTEND_H

#include<ros/ros.h>
#include<string>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include "sensors.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;

namespace MCVIO
{
    class MCVIOfrontend
    {
    public:
        explicit MCVIOfrontend(string config_file);

        void 
        setUpROS(ros::NodeHandle *pub_node, ros::NodeHandle *private_node);

        void 
        addSensors(cv::FileStorage &fsSettings, ros::NodeHandle *private_node);

        void
        addMonocular(cv::FileNode &fsSettings, ros::NodeHandle *private_node);
    public:
        ros::NodeHandle *pub_node_;
        ros::NodeHandle *private_node_;

        ros::Publisher pub_img, pub_match;
        ros::Publisher pub_restart;

        string config_file;

        unordered_map<string, int> sensors_tag;
        unordered_map<string, int> tracker_tag;

        vector<std::shared_ptr<MCVIOsensor>> sensors;

        FrontEndResultsSynchronizer synchronizer;

        bool compressedType;
    };

    inline void
    img_callback(const sensor_msgs::Image::ConstPtr color_msg, MCVIOfrontend *frontend)
    {
        // frontend->processImage(color_msg);
    };

    inline void
    Compressedimg_callback(const sensor_msgs::CompressedImage::ConstPtr color_msg, MCVIOfrontend *frontend)
    {
        // frontend->processImage(color_msg);
    };

    inline MCVIOfrontend* MCVIOfrontend_;
}



#endif
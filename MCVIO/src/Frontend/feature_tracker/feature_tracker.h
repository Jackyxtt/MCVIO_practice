#pragma once

#include <iostream>
#include <queue>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"


#include "trackerbase.h"
#include "../../utility/tic_toc.h"
#include "../sensors.h"

#include <ros/ros.h>

#include "../../utility/utility.h"
#include "../../utility/Twist.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

namespace MCVIO
{
    class FeatureTracker: public TrackerBase
    {
    public:
        FeatureTracker();

        ~FeatureTracker();

        void readImage(const cv::Mat &_img, double _cur_time);

        void setMask();

        void addPoints();

        bool updateID(unsigned int i);

        void readIntrinsicParameter(const string &calib_file);

        void showUndistortion(const string &name);

        void rejectWithF();

        void undistortedPoints();

        bool inBorder(const cv::Point2f &pt);

        void getPt(int idx, int &id, geometry_msgs::Point32 &p, cv::Point2f &p_uv, cv::Point2f &v);

        void getCurPt(int idx, cv::Point2f &cur_pt);

        cv::Mat mask;
        cv::Mat fisheye_mask;

        cv::Mat prev_img, cur_img, forw_img;

        vector<cv::Point2f> n_pts;
        vector<cv::Point2f> prev_pts, cur_pts, forw_pts; //像素坐标
        vector<cv::Point2f> prev_un_pts, cur_un_pts; //去畸变后的归一化坐标
        vector<cv::Point2f> pts_velocity;

        map<int, cv::Point2f> cur_un_pts_map;
        map<int, cv::Point2f> prev_un_pts_map;

        double cur_time;
        double prev_time;

        std::shared_ptr<MCVIO::MCVIOcamera> cam;

    };

    extern ofstream log_tracking_point; 
}
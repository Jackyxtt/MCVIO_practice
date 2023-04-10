#pragma once

#include <opencv2/opencv.hpp>
#include <opencv/huigui/highgui.hpp>

#include <Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"


namespace MCVIO
{
    class TrackerBase
    {
    public:
        TrackerBase();
        virtual ~TrackerBase();
        // virtual void readImage(const cv::Mat &_img, const cv::Mat &_depth, double _cur_time);
        virtual void readImage(const cv::Mat &_img, double _cur_time);

        virtual void setMask();

        virtual void addPoints();

        virtual bool updateID(unsigned int i);

        virtual void readIntrinsicParameter(const string &calib_file);

        virtual void showUndistortion(const string &name);

        virtual void rejectWithF();

        virtual void undistortedPoints();

        virtual void getPt(int idx, int &id, geometry_msgs::Point32 &p, cv::Point2f &p_uv, cv::Point2f &v);
        virtual void getCurPt(int idx, cv::Point2f &cur_pt);

        virtual void Lock();
        virtual void Unlock();

        void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
        void reduceVector(vector<int> &v, vector<uchar> status);

        vector<int> ids;
        vector<int> track_cnt;
        camodocal::CameraPtr m_camera;
        int n_id;
        double FOCAL_LENGTH = 460.0;
    };
} // namespace MCVIO
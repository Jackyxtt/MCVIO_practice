#pragma once

#include <vector>
#include <fstream>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <ros/ros.h>

// #include "../utility/utility.h"
// #include "../utility/Twist.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/CameraFactory.h"

// Estimator Parameters start
// -----------------
const double FOCAL_LENGTH = 460.0;

const int WINDOW_SIZE = 10;

extern int NUM_OF_CAM;

const int NUM_OF_F = 1000;

extern camodocal::CameraPtr m_camera;

//?:为什么要在此处定义
#define UNIT_SPHERE_ERROR

extern double INIT_DEPTH;

extern double MIN_PARALLAX;

extern double ACC_N, ACC_W;

extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC;

extern std::vector<Eigen::Vector3d> TIC;

extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;

extern double BIAS_GYR_THRESHOLD;

extern double SOLVER_TIME;

extern int NUM_ITERATIONS;

extern int NUM_OF_CAM;

extern int ESTIMATE_EXTRINSIC;

extern int ESTIMATE_TD;

extern int ROLLING_SHUTTER;

extern std::string EX_CALIB_RESULT_PATH;

extern std::string VINS_RESULT_PATH;

extern std::string IMU_TOPIC;

extern double TD, TR;

// FeatureTracking Parameters start
// image, lidar message topic
extern std::string IMAGE_TOPIC;

extern std::string DEPTH_TOPIC;

extern std::string VINS_World_Frame;

extern std::string Camera_Frame;

extern std::string VINS_IMU_Frame;

extern int LASER_TYPE;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name);

void readParameters(ros::NodeHandle &n);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};
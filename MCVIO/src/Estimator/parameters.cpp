#include "parameters.h"
#include <gflags/gflags.h>
#include <glog/logging.h>

camodocal::CameraPtr m_camera;

double INIT_DEPTH;

double MIN_PARALLAX;

double ACC_N, ACC_W;

double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;

std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;

double BIAS_GYR_THRESHOLD;

double SOLVER_TIME;

int NUM_ITERATIONS;

int NUM_OF_CAM;

int ESTIMATE_EXTRINSIC;

int ESTIMATE_TD;

int ROLLING_SHUTTER;

std::string EX_CALIB_RESULT_PATH;

std::string VINS_RESULT_PATH;

std::string IMU_TOPIC;

double TD, TR;

// FeatureTracking Parameters start
// image, lidar message topic
std::string IMAGE_TOPIC;

std::string DEPTH_TOPIC;

std::string VINS_World_Frame;

std::string Camera_Frame;

std::string VINS_IMU_Frame;

int LASER_TYPE;

//从launch文件中读取
template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans)){
        LOG(INFO) << "Loaded " << name << ": " << ans;
    }
    else
    {
        LOG(ERROR) << "Failed to load " << name;
        n.shutdown();
    }
    return ans;

}

void readParameters(ros::NodeHandle &n)
{
    LOG(INFO) << "[vins estimator] read parameter";
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    LOG(INFO) << "[vins estimator] config_file: " << config_file;
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);

    if (!fsSettings.isOpened()){
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    // read FeatureTracking config start
    std:: string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");
    LOG(INFO) << "\nvins_folder" << VINS_FOLDER_PATH;

    VINS_World_Frame = readParam<std::string>(n, "vins_world_frame");
    // Camera_Frame = readParam<std::string>(n, "camera_frame");
    VINS_IMU_Frame = readParam<std::string>(n, "vins_imu_frame");
    Camera_Frame = readParam<std::string>(n, "camera_frame");

    //字符串的读取用>>流处理
    fsSettings["imu_topic"] >> IMU_TOPIC;

    //int的读取用=赋值
    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    std::string OUTPUT_PATH;
    fsSettings["output_path"] >> OUTPUT_PATH;
    VINS_RESULT_PATH = OUTPUT_PATH + "/vins_result_no_loop.txt";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];

    if (ESTIMATE_EXTRINSIC == 2){
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
    }
    else
    {
        if (ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");
    }

    //?:这三个数的作用是什么
    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];

    ESTIMATE_TD = fsSettings["estimate_td"];

    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = fsSettings["rolling_shutter_tr"];
        ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << TR);
    }
    else
    {
        TR = 0;
    }

    fsSettings.release();

}
#include <ros/ros.h>
#include <thread>
#include <glog/logging.h>
#include "Estimator/parameters.h"
#include "Frontend/MCVIOfrontend.h"
#include "Estimator/MCVIOestimator.h"

using namespace std;
using namespace MCVIO;

void process1(){

}

int main(int argc, char **argv){
    ros::init(argc, argv, "MCVIO_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    string log_dir;
    n.getParam("log_dir", log_dir);
    FLAGS_log_dir = log_dir;
    FLAGS_alsologtostderr = 1;
    google::InitGoogleLogging("MCVIO_practice");

    readParameters(n);

    string config_file = readParam<string>(n, "config_file");

    MCVIOfrontend frontend(config_file);
    MCVIOfrontend_ = &frontend;

    frontend.setUpROS(nullptr, &n);
    // LOAD TIC RIC
    NUM_OF_CAM = (int)frontend.tracker_tag.size();
    LOG(INFO) << "NUM_OF_CAM:" << NUM_OF_CAM;
    TIC.resize(NUM_OF_CAM);
    RIC.resize(NUM_OF_CAM);
    TIC.clear();
    RIC.clear();
    std::vector<std::string> names;
    names.resize(NUM_OF_CAM);
    for (auto i : frontend.tracker_tag)
    {
        std::string name = i.first;
        int idx = i.second;

        auto sensor = frontend.sensors[frontend.sensors_tag[name]];
        TIC[idx] = sensor->ext_T;
        RIC[idx] = sensor->ext_R;
        names[idx] = name;
    }
    for (int c = 0; c < NUM_OF_CAM; c++)
    {
        LOG(INFO) << names[c];
        LOG(INFO) << "EXT_R: \n"
                  << RIC[c];
        LOG(INFO) << "EXT_T: \n"
                  << TIC[c];
    }

    // estimator.init(&frontend);
    
    ROS_WARN("waiting for image and imu...");
    LOG(INFO) << "Register publishers";
    // registerPub(n); 记录轨迹
    LOG(INFO) << "Finish initialization";
    
    std::thread measurement_process{process1};

    ros::MultiThreadedSpinner spinner(3);

    spinner.spin();
    google::ShutdownGoogleLogging();

    return 0;

}
#include <ros/ros.h>
#include <glog/logging.h>
#include "Estimator/parameters.h"
#include "Frontend/MCVIOfrontend.h"

using namespace std;
using namespace MCVIO;

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
    google::ShutdownGoogleLogging();

}
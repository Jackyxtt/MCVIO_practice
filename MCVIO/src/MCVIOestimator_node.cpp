#include <ros/ros.h>
#include "Estimator/parameters.h"

using namespace std;
using namespace MCVIO;

int main(int argc, char **argv){
    ros::init(argc, argv, "MCVIO_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);

    string config_file = readParam<string>(n, "config_file");

    MCVIOfrontend frontend(config_file);
    MCVIOfrontend_ = &frontend;
    frontend.setUpROS(nullptr, &n);

}
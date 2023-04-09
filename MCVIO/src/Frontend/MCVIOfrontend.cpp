#include "MCVIOfrontend.h"
#include "Frontend/sensors.h"

using namespace MCVIO;
using namespace std;

MCVIOfrontend::MCVIOfrontend(string config_file){
    this->config_file = config_file;
}


void MCVIOfrontend::setUpROS(ros::NodeHandle *pub_node, ros::NodeHandle *private_node){
    LOG(INFO) << "Setup ROS";
    this->pub_node_ = pub_node;
    this->private_node_ = private_node;
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    pub_restart = private_node_->advertise<std::msgs::Bool>("restart", 1000);
    addSensors(fsSettings, private_node);
}

void MCVIOfrontend::addSensors(cv::FileStorage &fsSettings, ros::NodeHandle *private_node){
    auto cam_list = fsSettings["sensor_list"];

    string imgMsg_type;
    fsSettings["imageMsg_type"] >> imgMsg_type;

    if(imgMsg_type == "Image")
        compressedType = false;
    else
        compressedType = true;

    vector<string> sensor_name_list;

    for(auto it = cam_list.begin(); it != cam_list.end(); it++){
        sensor_name_list.push_back(string(*it));
    }

    LOG(INFO) << "Sensor numbers:" << sensor_name_list.size();
    for (auto i : sensor_name_list)
    {
        LOG(INFO) << i;
    }

    for (auto i:sensor_name_list){

        int sensor = fsSettings[i]["sensor_type"];
        LOG(INFO) << "sensor type:" << i << "," << sensor;
        if (sensor == sensor_type::MONOCULAR)
        {
            synchronizer.addPool(i);
            cv::FileNode fsmono = fsSettings[i];
            addMonocular(fsmono, private_node);
            continue;
        }

    }

    
}

void MCVIOfrontend::addMonocular(cv::FileNode &fsSettings, ros::NodeHandle *private_node){
    LOG(INFO) << "Add monocular";
    cv::Mat cv_R, cv_T;
    Eigen::Matrix3d R, T;

    fsSettings["extrinsicRotation_imu_camera"] >> cv_R;
    fsSettings["extrinsicTranslation_imu_camera"] >> cv_T;

    cv::cv2eigen(cv_R, R);
    cv::cv2eigen(cv_T, T);

    LOG(INFO) << "Extrinsic rotation:\n"
              << R;
    LOG(INFO) << "Extrinsic translation:\n"
              << T;
    string topic, name;
    fsSettings["left_image_topic"] >> topic;
    fsSettings["frame_id"] >> name;
    LOG(INFO) << name << " subscribe to " << topic;

    int row, col;
    double fx, fy, cx, cy;
    bool fisheye;

    row = fsSettings["image_height"];
    col = fsSettings["image_width"];
    fsSettings["projection_parameters"]["fx"] >> fx;
    fsSettings["projection_parameters"]["fy"] >> fy;
    fsSettings["projection_parameters"]["cx"] >> cx;
    fsSettings["projection_parameters"]["cy"] >> cy;
    fsSettings["fisheye"] >> fisheye;
    LOG(INFO) << "Image size:[row,col] = [" << row << "," << col << "]";
    LOG(INFO) << "Use fisheye mask? : " << fisheye;

    std::shared_ptr<MCVIOcamera> monocam = 
        std::make_shared<MCVIOcamera>(MCVIOcamera(sensor_type::MONOCULAR,
                                                  topic,
                                                  name,
                                                  private_node,
                                                  R, T,
                                                  fx, fy, cx, cy, fisheye,
                                                  col, row, compressedType));


    if (fisheye)
    {
        string fisheye_path;
        fsSettings["fisheye_path"] >> fisheye_path;
        monocam->setFisheye(fisheye_path);
    }
}

FrontEndResultsSynchronizer::FrontEndResultsSynchronizer(){
}

void FrontEndResultsSynchronizer::addPool(std::string cam_name){
    LOG(INFO) << "add pool:" << cam_name;
    result_mutexes.push_back(std::make_shared<std::mutex>());
    results.push_back(std::make_shared<std::queue<std::shared_ptr<CameraProcessingResults>>>());
}

MCVIOsensor::MCVIOsensor(sensor_type stype,
                         string topic,
                         string name,
                         ros::NodeHandle *node,
                         Eigen::Matrix3d R,
                         Eigen::Vector3d T)
{
    type = stype;
    this->topic = topic;
    frontend_node = node;
    ext_R = R;
    ext_T = T;
    this->name = name;
}

MCVIOcamera(sensor_type type,
            string topic,
            ros::NodeHandle *node,
            Eigen::Matrix3d R,
            Eigen::Vector3d T,
            double fx, double fy, double cx, double cy, bool fisheye,
            int w, int h, bool compressedType) : MCVIOsensor(type, topic, name, node, R, T)
{
    this->fx = fx;
    this->fy = fy;
    this->cx = cx;
    this->cy = cy;
    FISHEYE = fisheye;
    this->COL = col;
    this->ROW = row;

    if(compressedType)
        sub = this->frontend_node->adverttise<sensor_msgs::CompressedImage>(topic, 5, boost::bind(&MCVIO::Compressedimg_callback, _1, MCVIOfrontend_));
}
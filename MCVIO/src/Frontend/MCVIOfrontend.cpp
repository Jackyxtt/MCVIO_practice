#include "MCVIOfrontend.h"
#include <glog/logging.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include "feature_tracker/feature_tracker.h"
#include "../Estimator/parameters.h"


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
    pub_restart = private_node_->advertise<std_msgs::Bool>("restart", 1000);
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
 
    synchronizer.tracker_tag = tracker_tag;
    
}

void MCVIOfrontend::addMonocular(cv::FileNode &fsSettings, ros::NodeHandle *private_node){
    LOG(INFO) << "Add monocular";
    cv::Mat cv_R, cv_T;
    Eigen::Matrix3d R;
    Eigen::Vector3d T;

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
    fsSettings["visualize"] >> monocam->visualize;
    monocam->init_visualization();
    fsSettings["max_cnt"] >> monocam->MAX_CNT;
    fsSettings["min_dist"] >> monocam->MIN_DIST;
    fsSettings["freq"] >> monocam->FREQ;
    fsSettings["F_threshold"] >> monocam->F_THRESHOLD;
    fsSettings["equalize"] >> monocam->EQUALIZE;
    LOG(INFO) << "Finish loading tracker parameters";
    if (sensors_tag.find(name) != sensors_tag.end())
    {
        LOG(INFO) << "Duplicated sensors. Check configureation file!";
        assert(sensors_tag.find(name) == sensors_tag.end());
    }

    // add new feature tracker and associate it with the camera
    monocam->tracker_idx = trackerData.size();
    // sensors_tag[name] = make_pair(sensors.size(), trackerData.size());
    sensors_tag[name] = sensors.size();

    sensors.push_back(monocam);

    // TODO:完善FeatureTracker类
    std::shared_ptr<FeatureTracker> tracker =
        std::make_shared<FeatureTracker>(FeatureTracker());

    tracker->cam = monocam;


    // register camera
    string config_file;
    fsSettings["camera_config_file"] >> config_file;
    tracker->readIntrinsicParameter(config_file);//tracker从yaml文件中读取相机参数
    LOG(INFO) << "Finish loading camera intrinsic to tracker";
    tracker->fisheye_mask = monocam->mask.clone();
    tracker_tag[name] = trackerData.size();
    trackerData.push_back(tracker);

}

FrontEndResultsSynchronizer::FrontEndResultsSynchronizer(){
}

void FrontEndResultsSynchronizer::addPool(std::string cam_name){
    LOG(INFO) << "add pool:" << cam_name;
    result_mutexes.push_back(std::make_shared<std::mutex>());
    results.push_back(std::make_shared<std::queue<std::shared_ptr<CameraProcessingResults>>>());
}

bool MCVIOcamera::setFisheye(string fisheye_path){
    mask = cv::imread(fisheye_path, 0);
    LOG(INFO) << "Fisheye mask path: " << fisheye_path;
    return true; 
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

MCVIOcamera::MCVIOcamera(sensor_type type,
            string topic,
            string name,
            ros::NodeHandle *node,
            Eigen::Matrix3d R,
            Eigen::Vector3d T,
            double fx, double fy, double cx, double cy, bool fisheye,
            int col, int row, bool compressedType) : MCVIOsensor(type, topic, name, node, R, T)
{
    this->fx = fx;
    this->fy = fy;
    this->cx = cx;
    this->cy = cy;
    FISHEYE = fisheye;
    this->COL = col;
    this->ROW = row;

    if(compressedType)
        sub = this->frontend_node->subscribe<sensor_msgs::CompressedImage>(topic, 5, boost::bind(&MCVIO::Compressedimg_callback, _1, MCVIOfrontend_));
    else
        sub = this->frontend_node->subscribe<sensor_msgs::Image>(topic, 5, boost::bind(&MCVIO::img_callback, _1, MCVIOfrontend_));

    mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));
    first_image_flag = true;
    first_image_time = 0;
    std::string match_img_topic = this->name + "/feature_image";
    pub_match = this->frontend_node->advertise<sensor_msgs::Image>(match_img_topic.c_str(), 5);
}

void MCVIOcamera::init_visualization(){
    
    pub_cam_pose = this->frontend_node->advertise<nav_msgs::Odometry>(this->name + "/camera_pose", 1000); 
    if (visualize){
        cameraposevisual = CameraPoseVisualization(0, 1, 0, 1);
        keyframebasevisual = CameraPoseVisualization(0, 0, 1, 1);

        cameraposevisual.setns(this->name + "_pose_visualization");
        cameraposevisual.setScale(1);
        cameraposevisual.setLineWidth(0.05);
        cameraposevisual.setns(this->name + "_KF_visualization");
        keyframebasevisual.setScale(1);
        keyframebasevisual.setLineWidth(0.05);

        pub_cam_pose_visual = this->frontend_node->advertise<visualization_msgs::MarkerArray>(this->name + "/camera_pose_visual", 1000);
        pub_slidewindow_camera_pose =
            this->frontend_node->advertise<visualization_msgs::MarkerArray>(this->name + "/slidewindow_pose", 1000);                
    }
}

void MCVIOfrontend::processImage(const sensor_msgs::CompressedImageConstPtr &color_msg)
{
    // step0: extract frame_id and find associate trackerData
    auto frame_id = color_msg->header.frame_id;
    int sensor_idx = sensors_tag[frame_id];
    int tracker_idx = tracker_tag[frame_id];
    std::shared_ptr<MCVIOsensor> sensor = sensors[sensor_idx]; //根据sensor_idx确认用哪个相机
    std::shared_ptr<MCVIOcamera> cam = dynamic_pointer_cast<MCVIOcamera>(sensor);
    auto &tracker = trackerData[tracker_idx];
#if SHOW_LOG_DEBUG
    LOG(INFO) << "Process image from:" << frame_id;
#endif
    // step1: first image, frequence control and format conversion
    if (cam->first_image_flag)
    {
#if SHOW_LOG_DEBUG
        LOG(INFO) << cam->name << ": first image [" << cam->ROW << "," << cam->COL << "]";
#endif
        cam->first_image_flag = false;
        cam->first_image_time = color_msg->header.stamp.toSec();
        cam->last_image_time = color_msg->header.stamp.toSec();
    }

    if (color_msg->header.stamp.toSec() - cam->last_image_time > 1.0 || color_msg->header.stamp.toSec() < cam->last_image_time)
    {
        ROS_WARN("Camera %d's image discontinue! reset the feature tracker!", sensor_idx);
        cam->first_image_flag = true;
        cam->last_image_time = 0;
        cam->pub_count = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;        
    }

    cam->last_image_time = color_msg->header.stamp.toSec();
    // frequency control
    // 到现在为止经过的帧数 / 时间 <= 帧率
    if (round(1.0 * cam->pub_count / (color_msg->header.stamp.toSec() - cam->first_image_time)) <= cam->FREQ)
    {
        cam->PUB_THIS_FRAME = true;
        // reset the frequency control 真实帧率相比于设定帧率过慢
        if (abs(1.0 * cam->pub_count / (color_msg->header.stamp.toSec() - cam->first_image_time) - cam->FREQ) < 0.01 * cam->FREQ)
        {
            cam->first_image_time = color_msg->header.stamp.toSec();
            cam->pub_count = 0;
        }
    }
    else
        cam->PUB_THIS_FRAME = false;    


    cv_bridge::CvImageConstPtr ptr;

    ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::MONO8);

    // step2: process image and achieve feature detection

    cv::Mat show_img = ptr->image;
    TicToc t_r;
    tracker->readImage(ptr->image.rowRange(0, cam->ROW), color_msg->header.stamp.toSec());

#if SHOW_UNDISTORTION

#endif
    // update all id in ids[]
    // If has ids[i] == -1 (newly added pts by cv::goodFeaturesToTrack), substitute by gloabl id counter (n_id)
    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        completed |= tracker->updateID(i); //TODO:为什么不直接判断tracker->updateID(i)，为false就结束循环
        if (!completed)
            break;
    }
    ROS_DEBUG("Complete update ids");

    // step3: assign depth for visual features
    if (cam->PUB_THIS_FRAME){
#if SHOW_LOG_DEBUG
        LOG(INFO)
            << "Pub this frame";
        ROS_DEBUG("Pub this frame");
#endif
        cam->pub_count++;    

        /// Publish FeatureTrack Result
        std::shared_ptr<CameraProcessingResults> output = std::make_shared<CameraProcessingResults>();
        output->timestamp = color_msg->header.stamp.toSec();

        // 3.2 publish featureTrack result
        synchronizer.result_mutexes[tracker_tag[frame_id]]->lock(); //有多少个相机，就有多少个tracker_tag，使用对应tracker_tag的互斥锁
        tracker->Lock();//该函数为空
        for (size_t j = 0; j < tracker->ids.size(); j++){
            if (tracker->track_cnt[j] > 1){
                geometry_msgs::Point32 p;
                cv::Point2f p_uv, v;
                int p_id;
                tracker->getPt(j, p_id, p, p_uv, v);

                cv::Point2f pts(p.x, p.y);
                Eigen::Matrix<double, 8, 1> xyz_uv_velocity;
                xyz_uv_velocity << p.x, p.y, p.z, p_uv.x, p_uv.y, v.x, v.y, -1;

                output->features[p_id].emplace_back(xyz_uv_velocity);
            }
        }
        tracker->Unlock();//该函数为空

#if SHOW_LOG_DEBUG
        LOG(INFO) << "Cam: " << frame_id << " feature size: " << output->features.size();
#endif        
        synchronizer.results[tracker_tag[frame_id]]->push(output);
        synchronizer.result_mutexes[tracker_tag[frame_id]]->unlock();        

        //skip the first image; since no optical speed on frist image
        if (!cam->init_pub){
            cam->init_pub = 1;
        }
        else
        {
            if (pub_img.getNumSubscribers() != 0){

            }
        }

        // step 4. Show image with tracked points in rviz (by topic pub_match)  
        if (SHOW_TRACK)
        {
            cv_bridge::CvImageConstPtr ptr1 = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);

            cv::Mat tmp_img = ptr1->image;

            for (size_t j = 0; j < tracker->ids.size(); j++){
                cv::Point2f p;
                tracker->getCurPt(j,p);
                double len = std::min(1.0, 1.0 * tracker->track_cnt[j] / WINDOW_SIZE);
                cv::circle(tmp_img, p, 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
            }

            // 修改tmp_img，也会导致ptr1->image改变    
            // cv::circle(tmp_img, {200, 200}, 30, cv::Scalar(255, 255, 255), 2);

            cam->pub_match.publish(ptr->toImageMsg());

        }
    }
}

void MCVIOfrontend::processImage(const sensor_msgs::ImageConstPtr &color_msg){
    // step0: extract frame_id and find associate trackerData
    auto frame_id = color_msg->header.frame_id;
    int sensor_idx = sensors_tag[frame_id];
    int tracker_idx = tracker_tag[frame_id];
    std::shared_ptr<MCVIOsensor> sensor = sensors[sensor_idx];
    std::shared_ptr<MCVIOcamera> cam = dynamic_pointer_cast<MCVIOcamera>(sensor);
    auto &tracker = trackerData[tracker_idx];
#if SHOW_LOG_DEBUG
    LOG(INFO) << "Process image from:" << frame_id;
#endif
    // step1: first image, frequence control and format conversion
    if (cam->first_image_flag)
    {
#if SHOW_LOG_DEBUG
        LOG(INFO) << cam->name << ": first image [" << cam->ROW << "," << cam->COL << "]";
#endif
        cam->first_image_flag = false;
        cam->first_image_time = color_msg->header.stamp.toSec();
        cam->last_image_time = color_msg->header.stamp.toSec();

    }
    // detect unstable camera stream
    if (color_msg->header.stamp.toSec() - cam->last_image_time > 1.0 || color_msg->header.stamp.toSec() < cam->last_image_time)
    {
        ROS_WARN("Camera %d's image discontinue! reset the feature tracker!", sensor_idx);
        cam->first_image_flag = true;
        cam->last_image_time = 0;
        cam->pub_count = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }

    cam->last_image_time = color_msg->header.stamp.toSec();
    // frequency control
    if (round(1.0 * cam->pub_count / (color_msg->header.stamp.toSec() - cam->first_image_time)) <= cam->FREQ)
    {
        cam->PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * cam->pub_count / (color_msg->header.stamp.toSec() - cam->first_image_time) - cam->FREQ) < 0.01 * cam->FREQ)
        {
            cam->first_image_time = color_msg->header.stamp.toSec();
            cam->pub_count = 0;
        }
    }
    else
        cam->PUB_THIS_FRAME = false;
    // encodings in ros: http://docs.ros.org/diamondback/api/sensor_msgs/html/image__encodings_8cpp_source.html
    // color has encoding RGB8
    cv_bridge::CvImageConstPtr ptr;
    if (color_msg->encoding == "8UC1")
    {
        // shan:why 8UC1 need this operation? Find answer:https://github.com/ros-perception/vision_opencv/issues/175
        sensor_msgs::Image img;
        img.header = color_msg->header;
        img.height = color_msg->height;
        img.width = color_msg->width;
        img.is_bigendian = color_msg->is_bigendian;
        img.step = color_msg->step;
        img.data = color_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        ROS_INFO("MONO_FORMAT!");
    }
    else
    {
        ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::MONO8);
    }
    cv::Mat rgb;

    cvtColor(ptr->image, rgb, cv::COLOR_GRAY2BGR);

    if (rgb.type() != CV_8UC3)
    {
        ROS_ERROR_STREAM("input image type != CV_8UC3");
    }
    // step2: process image and achieve feature detection

    cv::Mat show_img = ptr->image;
    TicToc t_r;
    tracker->readImage(ptr->image.rowRange(0, cam->ROW), color_msg->header.stamp.toSec());
    // always 0
#if SHOW_UNDISTORTION
    tracker->.showUndistortion("undistrotion_" + std::to_string(i));
    // }
    ROS_DEBUG("Finish processing tracker data");
#endif
    // update all id in ids[]
    // If has ids[i] == -1 (newly added pts by cv::goodFeaturesToTrack), substitute by gloabl id counter (n_id)
    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        completed |= tracker->updateID(i);
        if (!completed)
            break;
    }
    ROS_DEBUG("Complete update ids");
    // step3: assign depth for visual features
    if (cam->PUB_THIS_FRAME)
    {
#if SHOW_LOG_DEBUG
        LOG(INFO)
            << "Pub this frame";
        ROS_DEBUG("Pub this frame");
#endif
        cam->pub_count++;

        /// Publish FeatureTrack Result
        // ROS_DEBUG("Init FrontEndResult output");
        std::shared_ptr<CameraProcessingResults> output = std::make_shared<CameraProcessingResults>();
        output->timestamp = color_msg->header.stamp.toSec();
        // ROS_DEBUG("Finish init %d", output->cloud_ptr->size());

        // 3.2 publish featureTrack result
        // cam->lidar_mutex.lock();
        synchronizer.result_mutexes[tracker_tag[frame_id]]->lock();
        tracker->Lock();
        // for (size_t j = 0; j < ids.size(); j++)
        for (size_t j = 0; j < tracker->ids.size(); j++)
        {
            if (tracker->track_cnt[j] > 1)
            {
                geometry_msgs::Point32 p;
                cv::Point2f p_uv, v;
                int p_id;
                tracker->getPt(j, p_id, p, p_uv, v);

                cv::Point2f pts(p.x, p.y);
                Eigen::Matrix<double, 8, 1> xyz_uv_velocity;
                xyz_uv_velocity << p.x, p.y, p.z, p_uv.x, p_uv.y, v.x, v.y, -1;
                // ROS_DEBUG("Set feature[%d] into output", p_id);
                // image[feature_id].emplace_back(camera_id,  xyz_uv_velocity_depth);
                output->features[p_id].emplace_back(xyz_uv_velocity);
            }
        }
        tracker->Unlock();
        // cam->lidar_mutex.unlock();
        // synchronizer.result_mutexes[tracker_tag[frame_id]]->unlock();
        // datamuex_->lock();
        // need to modify fontend_output_queue
#if SHOW_LOG_DEBUG
        LOG(INFO) << "Cam: " << frame_id << " feature size: " << output->features.size();
#endif
        synchronizer.results[tracker_tag[frame_id]]->push(output);
        synchronizer.result_mutexes[tracker_tag[frame_id]]->unlock();
        // fontend_output_queue->push(output);
        // datamuex_->unlock();
#if MERGELASER
        // visualize features in cartesian 3d space (including the feature without depth (default 1))
        if (pub_depth_points.getNumSubscribers() != 0)
            publishCloud(&pub_depth_points, features_3d_sphere, feature_points->header.stamp, Laser_Frame);
#endif
        // skip the first image; since no optical speed on frist image
        if (!cam->init_pub)
        {
            cam->init_pub = 1;
        }
        else
        {
            if (pub_img.getNumSubscribers() != 0)
            {
                // ROS_DEBUG("publish");
                // pub_img.publish(feature_points); //"feature"
            }
        }
        // step 4. Show image with tracked points in rviz (by topic pub_match)
        if (SHOW_TRACK)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
            // cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            cv::Mat tmp_img = stereo_img.rowRange(0, 1 * cam->ROW);

            cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB); //??seems useless?
            tracker->Lock();
            for (size_t j = 0; j < tracker->ids.size(); j++)
            {
                cv::Point2f p;
                tracker->getCurPt(j, p);
                double len = std::min(1.0, 1.0 * tracker->track_cnt[j] / WINDOW_SIZE);
                // cv::circle(tmp_img, tracker->cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                cv::circle(tmp_img, p, 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
            }
            tracker->Unlock();

            cam->pub_match.publish(ptr->toImageMsg());
        }
        // }
        // ROS_INFO("whole feature tracker processing costs: %f", t_r.toc());
    }
}
#include "feature_tracker.h"

using namespace MCVIO;

// 在原来的代码中，FeatureTracker::FeatureTracker() : TrackerBase()
// 考虑到构造函数无法继承，这里不加冒号试试
FeatureTracker::FeatureTracker(){
    n_id = 0;
}

//读取传入图像，通过光流法追踪上一帧的特征点，特征点去畸变
void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time){
    cv::Mat img;
    TicToc t_r;
    cur_time = _cur_time;

    if(cam->EQUALIZE){
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        ROS_DEBUG("CLAHE cost: %fms", t_c.toc());
    }else{
        img = _img;
    }

    //考虑第0帧率的情况
    if(forw_img.empty()){
        prev_img = cur_img = forw_img = img;
    }
    else//正常情况下，捕获的帧为forw_img
    {
        forw_img = img;
    }

    forw_pts.clear();

    //当前情况下含有的特征点（未考虑新进来的帧）
    if(cur_pts.size() > 0){
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::size(21,21), 3);
        for(int i = 0; i < int(forw_pts.size()); i++){
            if(status[i] && !inBorder(forw_pts[i]))
               status[i] = 0;
        }       
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status);

        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
        
    }

    for (auto &n : track_cnt)
        n++;

    if (cam->PUB_THIS_FRAME)
    {
        rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = cam->MAX_CNT - static_cast<int>(forw_pts.size());

    }

}

void setMask();

void addPoints();

bool updateID(unsigned int i);

void FeatureTracker::readIntrinsicParameter(const string &calib_file){
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void showUndistortion(const string &name);

void FeatureTracker::rejectWithF(){
    if(forw_pts.size() >= 8){
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        //求归一化平面坐标
        for (unsigned int i = 0; i < cur_pts.size(); i++){
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + cam->COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + cam->ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + cam->COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + cam->ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, cam->F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status); 
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());               
    }
}

void undistortedPoints();

bool inBorder(const cv::Point2f &pt);

void getPt(int idx, int &id, geometry_msgs::Point32 &p, cv::Point2f &p_uv, cv::Point2f &v);

void getCurPt(int idx, cv::Point2f &cur_pt);

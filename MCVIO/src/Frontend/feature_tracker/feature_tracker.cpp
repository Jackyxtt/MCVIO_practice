#include "feature_tracker.h"

using namespace MCVIO;

// 在原来的代码中，FeatureTracker::FeatureTracker() : TrackerBase()
// 考虑到构造函数无法继承，这里不加冒号试试
FeatureTracker::FeatureTracker(){
    n_id = 0;//特征点编号，遇到新特征点就进行自增
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
        // forw_pts的数量和cur_img一样,通过status数组表示是否追踪成功
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
        if (n_max_cnt > 0){
            if(mask.empty())
                cout << "mask is empty " << endl;
            if(mask.type() != CV_8UC1)
                cout << "mask type wrong" << endl;
            if(mask.size() != forw_img.size())
                cout << "wrong size" << endl;
            // n_pts是新提取的特征点
            cv::goodFeaturesToTrack(forw_img, n_pts, cam->MAX_CNT - forw_pts.size(), 0.01, cam->MIN_DIST, mask);
        }
        else
            n_pts.clear();
        
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;
        addPoints();
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());        
    }

    prev_img = cur_img;

    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    cur_img = forw_img;

    cur_pts = forw_pts;
    undistortedPoints();
    prev_time = cur_time;

}

void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}

void setMask();

bool FeatureTracker::updateID(unsigned int i){
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

void FeatureTracker::readIntrinsicParameter(const string &calib_file){
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void FeatureTracker::showUndistortion(const string &name){

}

void FeatureTracker::rejectWithF(){
    if(forw_pts.size() >= 8){
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        //求归一化平面坐标
        for (unsigned int i = 0; i < cur_pts.size(); i++){
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            // ??为什么是FOCAL_LENGTH乘，FOCAL_LENGTH在此处一直是460
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

void FeatureTracker::undistortedPoints(){
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    
    for(unsigned int i = 0; i < cur_pts.size(); i++){
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;

        m_camera->liftProjective(a, b);
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));

    }

    // calculate points velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1)
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map = cur_un_pts_map;
}

bool FeatureTracker::inBorder(const cv::Point2f &pt){
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < cam->COL - BORDER_SIZE &&
           BORDER_SIZE <= img_y && img_y < cam->ROW - BORDER_SIZE;
}

void FeatureTracker::getPt(int idx, int &id, geometry_msgs::Point32 &p, cv::Point2f &p_uv, cv::Point2f &v){
    id = ids[idx];
    p.x = cur_un_pts[idx].x;
    p.y = cur_un_pts[idx].y;
    p.z = 1;
    p_uv = cur_pts[idx];
    v = pts_velocity[idx];
}

void FeatureTracker::getCurPt(int idx, cv::Point2f &cur_pt){
    cur_pt = cur_pts[idx];
}

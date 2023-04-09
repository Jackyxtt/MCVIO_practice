#include<ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <queue>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;

namespace MCVIO{

    enum sensor_type
    {
        MONOCULAR,
        UNKNOWN
    };

    struct CameraProcessingResults
    {
        double timestamp;
        // TODO:add FeatureTrackerResults
        // FeatureTrackerResults features;
    };

    class FrontEndResultsSynchronizer
    {
    public:
        FrontEndResultsSynchronizer();
        ~FrontEndResultsSynchronizer(){};
        bool Sync();
        void addPool(std::string cam_name);
        void resize(size_t size);
        double timestamp;

        typedef std::shared_ptr<FrontEndResultsSynchronizer> Ptr;
        std::vector<std::shared_ptr<std::mutex>> result_mutexes;
        std::vector<std::shared_ptr<std::queue<std::shared_ptr<CameraProcessingResults>>>> results;
        // std::vector<std::pair<string, std::shared_ptr<CameraProcessingResults>>> current_result;
        unordered_map<string, int> tracker_tag;
        // TODO::add SyncCameraProcessingResults
        // std::queue<std::shared_ptr<SyncCameraProcessingResults>> sync_results;
    };

    class MCVIOsensor
    {
    public:
        MCVIOsensor(sensor_type stype,
                    string topic,
                    string name,
                    ros::NodeHandle *node,
                    Eigen::Matrix3d R,
                    Eigen::Vector3d T);
        virtual ~MCVIOsensor(){};

        // virtual void* ptr() = 0;

        sensor_type type;
        string topic, name;
        ros::Subscriber sub;
        ros::NodeHandle *frontend_node;
        Eigen::Matrix3d ext_R; // extrinsic rotation
        Eigen::Vector3d ext_T; // extrinsic translation
    };

    class MCVIOcamera: public MCVIOsensor
    {
    
    public:
        MCVIOcamera(sensor_type type,
                    string topic,
                    ros::NodeHandle *node,
                    Eigen::Matrix3d R,
                    Eigen::Vector3d T,
                    double fx, double fy, double cx, double cy, bool fisheye,
                    int w, int h, bool compressedType);
        

        virtual ~MCVIOcamera(){};

        bool setFisheye(string fisheye_path);
        void init_visualization();

        void pub_cam(Eigen::Vector3d & P, Eigen::Matrix3d & R, std_msgs::Header &header);
        
        bool FISHEYE;
        double fx, fy, cx, cy;
        int ROW, COL;
        cv::Mat mask;

        int MAX_CNT = 200;
        int MIN_DIST = 20;
        int FREQ = 10;

        double F_THRESHOLD = 1.0;
        bool EQUALIZE = 1;
        bool USE_VPI;
        int tracker_idx;

        bool first_image_flag = false;
        double first_image_time, last_image_time;
        int pub_count = 0;
        bool PUB_THIS_FRAME = false;
        int init_pub = 0;

        ros::Publisher pub_match;

        // For visualization 
        bool visualize = true;
        ros::Publisher pub_cam_pose, pub_cam_pose_visual, pub_slidewindow_camera_pose;
        
        // TODO:按照原来的代码加上本函数
        // CameraPoseVisualization cameraposevisual, keyframebasevisual;

    };


}
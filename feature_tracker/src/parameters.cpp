#include "parameters.h"
#include <opencv2/core/eigen.hpp>

std::string IMAGE_TOPIC;
std::string IMAGE_LEFT;
std::string IMAGE_RIGHT; 
std::string EVENT_TOPIC;
std::string EVENT_LEFT;
std::string EVENT_RIGHT;
std::string IMU_TOPIC;
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int use_stereo=0;
int use_stereo_event=0;
int STEREO = 1;
int system_mode = 0;// 0 for ESIO, 1 for ESVIO
Eigen::Matrix3d Eeesntial_matrix;
Eigen::Matrix3d Eeesntial_matrix_event;
int MAX_CNT;
int MAX_CNT_IMG;
int MIN_DIST;
int MIN_DIST_IMG;
int WINDOW_SIZE;
int FREQ;
int FREQ_IMG;
double F_THRESHOLD;

double TS_LK_THRESHOLD;//time surface threshold
int para_ignore_polarity;//whether use polarity
double para_decay_ms;//time surface decay
double para_decay_loop_ms;
int para_median_blur_kernel_size;
double para_feature_filter_threshold;

int SHOW_TRACK;
int FLOW_BACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int ROW_event;
int COL_event;
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;
int Num_of_thread;

int Do_motion_correction=0;

double fx,fy,cx,cy;
double fx_event,fy_event,cx_event,cy_event;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

namespace {
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
    {
            Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
            ans << typename Derived::Scalar(0), -q(2), q(1),
                q(2), typename Derived::Scalar(0), -q(0),
                -q(1), q(0), typename Derived::Scalar(0);
            return ans;
    }
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    std::string ESVIO_FOLDER_PATH = readParam<std::string>(n, "esvio_folder");

    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["image_left_topic"] >> IMAGE_LEFT;
    fsSettings["image_right_topic"] >> IMAGE_RIGHT; 
    fsSettings["event_topic"] >> EVENT_TOPIC;
    fsSettings["event_left_topic"] >> EVENT_LEFT;
    fsSettings["event_right_topic"] >> EVENT_RIGHT; 
    fsSettings["imu_topic"] >> IMU_TOPIC; 
    MAX_CNT = fsSettings["max_cnt"];
    MAX_CNT_IMG = fsSettings["max_cnt_img"];
    MIN_DIST = fsSettings["min_dist"];
    MIN_DIST_IMG = fsSettings["min_dist_img"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROW_event = fsSettings["event_height"];
    COL_event = fsSettings["event_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    Num_of_thread=fsSettings["Num_of_thread"];

    TS_LK_THRESHOLD=fsSettings["TS_LK_threshold"];
    para_ignore_polarity= fsSettings["ignore_polarity"];
    para_decay_ms= fsSettings["decay_ms"];
    para_median_blur_kernel_size= fsSettings["median_blur_kernel_size"];
    para_feature_filter_threshold=fsSettings["feature_filter_threshold"];

    Do_motion_correction=fsSettings["Do_motion_correction"];

    fx = fsSettings["fx"];
    fy = fsSettings["fy"];
    cx = fsSettings["cx"];
    cy = fsSettings["cy"];

    SHOW_TRACK = fsSettings["show_track"];
    FLOW_BACK = fsSettings["flow_back"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];
    if (FISHEYE == 1)
        FISHEYE_MASK = ESVIO_FOLDER_PATH + "/fisheye_mask.jpg";

    system_mode = fsSettings["system_mode"];// 0 for ESIO, 1 for ESVIO

        int STEREO = 1;

        std::string cam_left_Calibfile, cam_right_Calibfile; 
        fsSettings["cam_left_calib"] >> cam_left_Calibfile; 
        fsSettings["cam_right_calib"] >> cam_right_Calibfile;

        std::string cam_left_Path = ESVIO_FOLDER_PATH + "/" + cam_left_Calibfile; 
        std::string cam_right_Path = ESVIO_FOLDER_PATH + "/" + cam_right_Calibfile;
        
        ROS_DEBUG("cam_left_Path: %s", cam_left_Path.c_str());
        ROS_DEBUG("cam_right_Path: %s", cam_right_Path.c_str());

        CAM_NAMES.push_back(cam_left_Path); 
        CAM_NAMES.push_back(cam_right_Path); 

        //get the transformation between two cameras
        {
        Eigen::Matrix3d Rlr;
        Eigen::Vector3d Tlr; 
        Eigen::Matrix3d Rrl;     // Trl 
        Eigen::Vector3d Trl; 
        cv::Mat cv_R, cv_T; 
        fsSettings["Rrl"] >> cv_R; 
        fsSettings["Trl"] >> cv_T; 
        cv::cv2eigen(cv_R, Rrl); //cv to eigen
        cv::cv2eigen(cv_T, Trl); 
        Eigen::Quaterniond qq(Rrl); 
        Rrl = qq.normalized();
        Rlr = Rrl.transpose(); 
        Tlr = - Rlr * Trl;  
        ROS_INFO_STREAM("Rrl: " << std::endl << Rrl); 
        ROS_INFO_STREAM("Trl: " << std::endl << Trl.transpose());  

        Eeesntial_matrix.setZero();
        Eeesntial_matrix = skewSymmetric(Trl) * Rrl;
        }
    
    WINDOW_SIZE = 20;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();


}


void readParameters_event(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    std::string ESVIO_FOLDER_PATH = readParam<std::string>(n, "esvio_folder");

    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["image_left_topic"] >> IMAGE_LEFT;
    fsSettings["image_right_topic"] >> IMAGE_RIGHT; 
    fsSettings["event_topic"] >> EVENT_TOPIC;
    fsSettings["event_left_topic"] >> EVENT_LEFT;
    fsSettings["event_right_topic"] >> EVENT_RIGHT; 
    fsSettings["imu_topic"] >> IMU_TOPIC; 
    MAX_CNT = fsSettings["max_cnt"];
    MAX_CNT_IMG = fsSettings["max_cnt_img"];
    MIN_DIST = fsSettings["min_dist"];
    MIN_DIST_IMG = fsSettings["min_dist_img"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROW_event = fsSettings["event_height"];
    COL_event = fsSettings["event_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    Num_of_thread=fsSettings["Num_of_thread"];

    TS_LK_THRESHOLD=fsSettings["TS_LK_threshold"];
    para_ignore_polarity= fsSettings["ignore_polarity"];//true;
    para_decay_ms= fsSettings["decay_ms"];//
    para_median_blur_kernel_size= fsSettings["median_blur_kernel_size"];//
    para_feature_filter_threshold=fsSettings["feature_filter_threshold"];

    Do_motion_correction=fsSettings["Do_motion_correction"];

    fx = fsSettings["fx"];
    fy = fsSettings["fy"];
    cx = fsSettings["cx"];
    cy = fsSettings["cy"];

    SHOW_TRACK = fsSettings["show_track"];
    FLOW_BACK = fsSettings["flow_back"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];
    if (FISHEYE == 1)
        FISHEYE_MASK = ESVIO_FOLDER_PATH + "/fisheye_mask.jpg";

     // transformation between stereo cams 


        int STEREO = 1;

        std::string event_left_Calibfile, event_right_Calibfile; 
        fsSettings["event_left_calib"] >> event_left_Calibfile; 
        fsSettings["event_right_calib"] >> event_right_Calibfile;

        std::string event_left_Path = ESVIO_FOLDER_PATH + "/" + event_left_Calibfile; 
        std::string event_right_Path = ESVIO_FOLDER_PATH + "/" + event_right_Calibfile;
        
        ROS_DEBUG("event_left_Path: %s", event_left_Path.c_str());
        ROS_DEBUG("event_right_Path: %s", event_right_Path.c_str());

        CAM_NAMES.push_back(event_left_Path); 
        CAM_NAMES.push_back(event_right_Path); 

        //get the transformation between two event cameras
        {
        Eigen::Matrix3d Rlr_event;
        Eigen::Vector3d Tlr_event;
        Eigen::Matrix3d Rrl_event;     // Trl 
        Eigen::Vector3d Trl_event; 
        cv::Mat cv_R_event, cv_T_event; 
        fsSettings["Rrl_event"] >> cv_R_event; 
        fsSettings["Trl_event"] >> cv_T_event; 
        cv::cv2eigen(cv_R_event, Rrl_event); 
        cv::cv2eigen(cv_T_event, Trl_event); 
        Eigen::Quaterniond qq_event(Rrl_event); 
        Rrl_event = qq_event.normalized();
        Rlr_event = Rrl_event.transpose(); 
        Tlr_event = - Rlr_event * Trl_event;  
        ROS_INFO_STREAM("Rrl_event: " << std::endl << Rrl_event); 
        ROS_INFO_STREAM("Trl_event: " << std::endl << Trl_event.transpose());  

        Eeesntial_matrix_event.setZero();
        Eeesntial_matrix_event = skewSymmetric(Trl_event) * Rrl_event;
        }

    WINDOW_SIZE = 20;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();

}
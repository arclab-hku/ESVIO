#pragma once
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

extern int ROW;
extern int COL;
extern int ROW_event;
extern int COL_event;
extern int FOCAL_LENGTH;
const int NUM_OF_CAM = 1; 
const int NUM_OF_CAM_stereo = 2;
extern int STEREO;
extern int system_mode;// 0 for ESIO, 1 for ESVIO


extern std::string IMAGE_TOPIC;
extern std::string IMAGE_LEFT;
extern std::string IMAGE_RIGHT; 
extern std::string EVENT_TOPIC;
extern std::string EVENT_LEFT;
extern std::string EVENT_RIGHT;
extern std::string IMU_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MAX_CNT_IMG;
extern int MIN_DIST;//DIST for event
extern int MIN_DIST_IMG;//for image
extern int WINDOW_SIZE;
extern int FREQ;//FREQ for event
extern int FREQ_IMG;
extern double F_THRESHOLD;
extern double TS_LK_THRESHOLD;
extern int para_ignore_polarity;//true;
extern double para_decay_ms;
extern double para_decay_loop_ms;
extern int para_median_blur_kernel_size;
extern double para_feature_filter_threshold;

extern int Do_motion_correction;//whether use motion correction

//intrinsics matrix
extern double fx;
extern double fy;
extern double cx;
extern double cy;

extern double fx_event;
extern double fy_event;
extern double cx_event;
extern double cy_event;


extern int SHOW_TRACK;
extern int FLOW_BACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;
extern Eigen::Matrix3d Eeesntial_matrix;// essential matrix from left and right camera
extern Eigen::Matrix3d Eeesntial_matrix_event;// essential matrix from left and right event

extern int Num_of_thread;//

void readParameters(ros::NodeHandle &n);
void readParameters_event(ros::NodeHandle &n);

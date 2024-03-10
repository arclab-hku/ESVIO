#pragma once

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

#define SQ(x) (((x)*(x)))

const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 10;
const int NUM_OF_CAM = 2; 
const int NUM_OF_CAM_stereo = 2; 
const int NUM_OF_F = 3000; 

extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;

// for stereo
extern std::vector<std::string> CAM_NAMES;
extern Eigen::Matrix3d Rlr;    
extern Eigen::Vector3d Tlr; 
extern Eigen::Matrix3d Rrl;     
extern Eigen::Vector3d Trl;
extern Eigen::Matrix3d Rlr_event;    
extern Eigen::Vector3d Tlr_event; 
extern Eigen::Matrix3d Rrl_event;     
extern Eigen::Vector3d Trl_event; 

extern int temp_T;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string ESVIO_RESULT_PATH;
extern std::string IMU_TOPIC;
extern double TD;
extern double TR;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;
extern double ROW, COL;
extern double ROW_EVENT, COL_EVENT;
extern int system_mode;

extern double nG;
extern bool g_use_sampson_model;
extern bool g_use_stereo_correction; 
extern bool g_opt_verbose; 


void readParameters(ros::NodeHandle &n);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1,
    SIZE_LINE = 4
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};

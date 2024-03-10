#pragma once

#include "parameters.h"
#include "feature_manager.h"
#include "utility/utility.h"
#include "utility/tic_toc.h"
#include "initial/solve_5pts.h"
#include "initial/initial_sfm.h"
#include "initial/initial_alignment.h"
#include "initial/initial_ex_rotation.h"
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include <ceres/ceres.h>
#include "factor/imu_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projection_factor.h"
#include "factor/projection_td_factor.h"
#include "factor/marginalization_factor.h"
#include "factor/projectionOneFrameTwoCamFactor.h"
#include "factor/projectionTwoFrameOneCamFactor.h"
#include "factor/projectionTwoFrameTwoCamFactor.h"

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>


class Estimator
{
  public:
    Estimator();

    void setParameter();//set para
    void ESIO_setParameter();//set para
    void readIntrinsicParameter(vector<string>& calib_file);//para input

    // interface
    void processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void Stereo_processVisual(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, // image features
                            const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &event, // event features
                            const std_msgs::Header &header);
    void Stereo_processEvent(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &event, // event features
                            const std_msgs::Header &header);                        
    void setReloFrame(double _frame_stamp, int _frame_index, vector<Vector3d> &_match_points, Vector3d _relo_t, Matrix3d _relo_r);

    // internal
    void clearState();
    void ESIO_clearState();

    bool initialStructure();
    bool ESIO_initialStructure();
    bool initialStructureStereo();//stereo initialization
    bool ESIO_initialStructureStereo();//ESIO initialization

    bool visualInitialAlign();
    bool EventInitialAlign();
    bool visualInitialAlignWithDepth();
    bool EventInitialAlignWithDepth();

    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    bool relativePoseHybrid(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    bool Event_relativePoseHybrid(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    
    void solveOdometry_esvio();
    void solveOdometry_esio();

    void slideWindow();
    void slideWindowNew();
    void slideWindowOld();

    void optimizationesvio();
    void optimizationesio();

    void stereo_vector2double();
    void stereo_double2vector3();

    void esio_vector2double();
    void esio_double2vector3();

    bool failureDetection();


    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    double frame_cnt_ = 0;
    double num_extrinsic_fix;

    int t_solve_count = 0;
    double t_solve_time = 0;
    int t_solve_zero = 0;

    SolverFlag solver_flag;
    MarginalizationFlag  marginalization_flag;
    Vector3d g;
    MatrixXd Ap[2], backup_A;
    VectorXd bp[2], backup_b;

    Matrix3d ric[NUM_OF_CAM_stereo*2];//camera to imu
    Vector3d tic[NUM_OF_CAM_stereo*2];//camera to imu
    vector<camodocal::CameraPtr> m_camera;

    Vector3d Ps[(WINDOW_SIZE + 1)];
    Vector3d Vs[(WINDOW_SIZE + 1)];
    Matrix3d Rs[(WINDOW_SIZE + 1)];
    Vector3d Bas[(WINDOW_SIZE + 1)];
    Vector3d Bgs[(WINDOW_SIZE + 1)];
    double td;

    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;
    std_msgs::Header Headers[(WINDOW_SIZE + 1)];

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;

    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    int frame_count;
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;

    FeatureManager f_manager;
    MotionEstimator m_estimator;
    InitialEXRotation initial_ex_rotation;

    bool first_imu;
    bool is_valid, is_key;
    bool failure_occur;

    vector<Vector3d> point_cloud;
    vector<Vector3d> margin_cloud;
    vector<Vector3d> key_poses;
    double initial_timestamp;


    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];//pose
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];//bias
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_EventFeature[NUM_OF_F][SIZE_FEATURE];

    double para_Ex_Pose[NUM_OF_CAM*2][SIZE_POSE];
    double para_Retrive_Pose[SIZE_POSE];
    double para_Td[1][1];
    double para_Tr[1][1];

    int loop_window_index;

    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;

    map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration;

    //relocalization variable
    bool relocalization_info;
    double relo_frame_stamp;
    double relo_frame_index;
    int relo_frame_local_index;
    vector<Vector3d> match_points;
    double relo_Pose[SIZE_POSE];
    Matrix3d drift_correct_r;
    Vector3d drift_correct_t;
    Vector3d prev_relo_t;
    Matrix3d prev_relo_r;
    Vector3d relo_relative_t;
    Quaterniond relo_relative_q;
    double relo_relative_yaw;
};

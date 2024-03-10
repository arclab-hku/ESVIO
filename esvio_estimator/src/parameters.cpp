#include "parameters.h"

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;
std::vector<Eigen::Matrix3d> RIC_event;
std::vector<Eigen::Vector3d> TIC_event;

std::vector<std::string> CAM_NAMES;//cam para
Eigen::Matrix3d Rrl; //left to right
Eigen::Vector3d Trl; 
Eigen::Matrix3d Rlr; //right to left
Eigen::Vector3d Tlr;
Eigen::Matrix3d Rrl_event; //left event to right
Eigen::Vector3d Trl_event;  
Eigen::Matrix3d Rlr_event; //right event to left
Eigen::Vector3d Tlr_event; 

int temp_T=0;//whether need T
int use_stereo=0;//whether use stereo
int temp_T_event=0;
int use_stereo_event=0;
int system_mode;//0:ESIO 1:ESViO

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string ESVIO_RESULT_PATH;//out put the result
std::string IMU_TOPIC;
double ROW, COL;
double ROW_EVENT, COL_EVENT;
double TD, TR;

// para for stereo
double nG = 1.;
bool g_use_sampson_model = false;
bool g_use_stereo_correction = true; 
bool g_opt_verbose = false; 

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

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["imu_topic"] >> IMU_TOPIC;

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    std::string OUTPUT_PATH;
    fsSettings["output_path"] >> OUTPUT_PATH;
    ESVIO_RESULT_PATH = OUTPUT_PATH + "/esvio_result_no_loop.csv";//output result
    std::cout << "result path " << ESVIO_RESULT_PATH << std::endl;

    // create folder if not exists
    FileSystemHelper::createDirectoryIfNotExists(OUTPUT_PATH.c_str());

    std::ofstream fout(ESVIO_RESULT_PATH, std::ios::out);
    fout.setf(std::ios::fixed, std::ios::floatfield);
    fout.precision(0);
    fout << "time" << ",";
    fout.precision(5);
    fout << "x" << ","
            << "y" << ","
            << "z" << ","
            << "qw" << ","
            << "qx" << ","
            << "qy" << ","
            << "qz" << ","
            << "vx" << ","
            << "vy" << ","
            << "vz" << "," << std::endl;
    fout.close();


    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROW_EVENT = fsSettings["event_height"];
    COL_EVENT = fsSettings["event_width"]; 
    ROS_INFO("ROW_Image: %f COL_Image: %f ", ROW, COL);
    ROS_INFO("ROW_Event: %f COL_EVENT: %f ", ROW_EVENT, COL_EVENT);

    system_mode = fsSettings["system_mode"];

    if ( system_mode == 1){//ESVIO
        ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
        if (ESTIMATE_EXTRINSIC == 2)
        {
            ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
            RIC.push_back(Eigen::Matrix3d::Identity());
            TIC.push_back(Eigen::Vector3d::Zero());
            EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";

        }
        else 
        {
            if ( ESTIMATE_EXTRINSIC == 1)//have initial extrinsic param
            {
                ROS_WARN(" Optimize extrinsic param around initial guess!");
                EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
            }
            if (ESTIMATE_EXTRINSIC == 0)//have fixed extrinsic param
                ROS_WARN(" fix extrinsic param ");

            cv::Mat cv_R, cv_T;
            cv::Mat cv_R_event, cv_T_event;
            fsSettings["extrinsicRotation"] >> cv_R;
            fsSettings["extrinsicTranslation"] >> cv_T;
            fsSettings["extrinsicRotation_event"] >> cv_R_event;//fix event to imu
            fsSettings["extrinsicTranslation_event"] >> cv_T_event;
            
            Eigen::Matrix3d eigen_R;
            Eigen::Vector3d eigen_T;
            Eigen::Matrix3d eigen_R_event;
            Eigen::Vector3d eigen_T_event;

            cv::cv2eigen(cv_R, eigen_R);
            cv::cv2eigen(cv_T, eigen_T);
            cv::cv2eigen(cv_R_event, eigen_R_event);
            cv::cv2eigen(cv_T_event, eigen_T_event);

            Eigen::Quaterniond Q(eigen_R);
            Eigen::Quaterniond Q_event(eigen_R_event);

            eigen_R = Q.normalized();
            eigen_R_event = Q_event.normalized();

            fsSettings["T_camera_imu"]>>temp_T;//if the input is imu to cam
            fsSettings["T_event_imu"]>>temp_T_event;//if the input is imu to event
            Eigen::Matrix3d R_imu_camera;
            Eigen::Vector3d T_imu_camera;
            Eigen::Matrix3d R_imu_event;
            Eigen::Vector3d T_imu_event;

            if(temp_T){//need transpose
                ROS_INFO("give an T to the extrinsic of cam ");
                R_imu_camera=eigen_R.inverse();//get imu to cam
                T_imu_camera=-R_imu_camera*eigen_T;
            }
            else{
                R_imu_camera=eigen_R;
                T_imu_camera=eigen_T;
            }

            if(temp_T_event){//event need transpose
                ROS_INFO("give an T to the extrinsic of event ");
                R_imu_event=eigen_R_event.inverse();
                T_imu_event=-R_imu_event*eigen_T_event;
            }
            else{
                R_imu_event=eigen_R_event;
                T_imu_event=eigen_T_event;
            }

            RIC.push_back(R_imu_camera);
            TIC.push_back(T_imu_camera);

            RIC.push_back(R_imu_event);
            TIC.push_back(T_imu_event);

            ROS_INFO_STREAM("Extrinsic_R : " << std::endl << RIC[0]);
            ROS_INFO_STREAM("Extrinsic_T : " << std::endl << TIC[0].transpose());
            ROS_INFO_STREAM("Extrinsic_R_event : " << std::endl << RIC[1]);
            ROS_INFO_STREAM("Extrinsic_T_event : " << std::endl << TIC[1].transpose());

        } 

            n.param("gravity_norm", nG, nG);
            n.param("use_sampson_model", g_use_sampson_model, g_use_sampson_model);
            n.param("use_stereo_correction", g_use_stereo_correction, g_use_stereo_correction);
            n.param("opt_verbose", g_opt_verbose, g_opt_verbose); 
            ROS_INFO("parameters.cpp: gravity_norm: %lf", nG);
            std::cout <<"parameters.cpp: "<< (g_use_sampson_model?"Yes use sampson model":"Not use sampson model")<<std::endl;
            std::cout <<"parameters.cpp: "<< (g_use_stereo_correction?"Yes use geometric correction":"Not use geometric correction")<<std::endl;

            // right cam to imu
            cv::Mat cv_TT;
            fsSettings["body_T_cam1"] >> cv_TT;
            Eigen::Matrix4d T;
            cv::cv2eigen(cv_TT, T);
            RIC.push_back(T.block<3, 3>(0, 0));
            TIC.push_back(T.block<3, 1>(0, 3));
            ROS_INFO_STREAM("Extrinsic_R2 : " << std::endl << RIC[2]);
            ROS_INFO_STREAM("Extrinsic_T2 : " << std::endl << TIC[2].transpose());

            // transformation between stereo cams 
            {
                cv::Mat cv_R, cv_T; 
                fsSettings["Rrl"] >> cv_R; 
                fsSettings["Trl"] >> cv_T; 
                cv::cv2eigen(cv_R, Rrl); 
                cv::cv2eigen(cv_T, Trl); 
                Eigen::Quaterniond qq(Rrl); 
                Rrl = qq.normalized();
                Rlr = Rrl.transpose(); 
                Tlr = - Rlr * Trl;  
                ROS_INFO_STREAM("Rrl: " << std::endl << Rrl); 
                ROS_INFO_STREAM("Trl: " << std::endl << Trl.transpose());  
            }

            //cam para
            {
                std::string ESVIO_FOLDER_PATH = readParam<std::string>(n, "esvio_folder");

                std::string cam_left_Calibfile, cam_right_Calibfile; 
                fsSettings["cam_left_calib"] >> cam_left_Calibfile; 
                fsSettings["cam_right_calib"] >> cam_right_Calibfile; 

                std::string cam_left_Path = ESVIO_FOLDER_PATH + cam_left_Calibfile; 
                std::string cam_right_Path = ESVIO_FOLDER_PATH + cam_right_Calibfile; 

                ROS_DEBUG("cam_left_Path: %s", cam_left_Path.c_str()); 
                ROS_DEBUG("cam_right_Path: %s", cam_right_Path.c_str());
                CAM_NAMES.push_back(cam_left_Path); 
                CAM_NAMES.push_back(cam_right_Path); 
            }

            //right event to imu
            cv::Mat cv_TT_event;
            fsSettings["body_T_event1"] >> cv_TT_event;
            Eigen::Matrix4d T_event;
            cv::cv2eigen(cv_TT_event, T_event);
            RIC.push_back(T_event.block<3, 3>(0, 0));
            TIC.push_back(T_event.block<3, 1>(0, 3));
            ROS_INFO_STREAM("Extrinsic_R2_event : " << std::endl << RIC[3]);
            ROS_INFO_STREAM("Extrinsic_T2_event : " << std::endl << TIC[3].transpose());

            // transformation between stereo events 
            {
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
            }

            // get event para
            {
                std::string ESVIO_FOLDER_PATH = readParam<std::string>(n, "esvio_folder");

                std::string event_left_Calibfile, event_right_Calibfile; 
                fsSettings["event_left_calib"] >> event_left_Calibfile; 
                fsSettings["event_right_calib"] >> event_right_Calibfile; 

                std::string event_left_Path = ESVIO_FOLDER_PATH + event_left_Calibfile; 
                std::string event_right_Path = ESVIO_FOLDER_PATH + event_right_Calibfile; 

                ROS_DEBUG("event_left_Path: %s", event_left_Path.c_str()); 
                ROS_DEBUG("event_right_Path: %s", event_right_Path.c_str());
                CAM_NAMES.push_back(event_left_Path); 
                CAM_NAMES.push_back(event_right_Path); 
            }

    }
    else if (system_mode == 0){//ESIO

        ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
        if (ESTIMATE_EXTRINSIC == 2)
        {
            ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
            RIC.push_back(Eigen::Matrix3d::Identity());
            TIC.push_back(Eigen::Vector3d::Zero());
            EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";

        }
        else 
        {
            if ( ESTIMATE_EXTRINSIC == 1)//have initial extrinsic param
            {
                ROS_WARN(" Optimize extrinsic param around initial guess!");
                EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
            }
            if (ESTIMATE_EXTRINSIC == 0)//have fixed extrinsic param
                ROS_WARN(" fix extrinsic param ");

            cv::Mat cv_R, cv_T;
            cv::Mat cv_R_event, cv_T_event;
            fsSettings["extrinsicRotation_event"] >> cv_R;
            fsSettings["extrinsicTranslation_event"] >> cv_T;
            fsSettings["extrinsicRotation_event"] >> cv_R_event;//fix event to imu
            fsSettings["extrinsicTranslation_event"] >> cv_T_event;
            
            Eigen::Matrix3d eigen_R;
            Eigen::Vector3d eigen_T;
            Eigen::Matrix3d eigen_R_event;
            Eigen::Vector3d eigen_T_event;

            cv::cv2eigen(cv_R, eigen_R);
            cv::cv2eigen(cv_T, eigen_T);
            cv::cv2eigen(cv_R_event, eigen_R_event);
            cv::cv2eigen(cv_T_event, eigen_T_event);

            Eigen::Quaterniond Q(eigen_R);
            Eigen::Quaterniond Q_event(eigen_R_event);

            eigen_R = Q.normalized();
            eigen_R_event = Q_event.normalized();

            fsSettings["T_camera_imu"]>>temp_T;//if the input is imu to cam
            fsSettings["T_event_imu"]>>temp_T_event;//if the input is imu to event
            Eigen::Matrix3d R_imu_camera;
            Eigen::Vector3d T_imu_camera;
            Eigen::Matrix3d R_imu_event;
            Eigen::Vector3d T_imu_event;

            if(temp_T){//need transpose
                ROS_INFO("give an T to the extrinsic of cam ");
                R_imu_camera=eigen_R.inverse();//get imu to cam
                T_imu_camera=-R_imu_camera*eigen_T;
            }
            else{
                R_imu_camera=eigen_R;
                T_imu_camera=eigen_T;
            }

            if(temp_T_event){//event need transpose
                ROS_INFO("give an T to the extrinsic of event ");
                R_imu_event=eigen_R_event.inverse();
                T_imu_event=-R_imu_event*eigen_T_event;
            }
            else{
                R_imu_event=eigen_R_event;
                T_imu_event=eigen_T_event;
            }

            RIC.push_back(R_imu_camera);
            TIC.push_back(T_imu_camera);

            RIC.push_back(R_imu_event);
            TIC.push_back(T_imu_event);

            ROS_INFO_STREAM("Extrinsic_R_event : " << std::endl << RIC[0]);
            ROS_INFO_STREAM("Extrinsic_T_event : " << std::endl << TIC[0].transpose());
            ROS_INFO_STREAM("Extrinsic_R_event : " << std::endl << RIC[1]);
            ROS_INFO_STREAM("Extrinsic_T_event : " << std::endl << TIC[1].transpose());

        } 


            n.param("gravity_norm", nG, nG);
            n.param("use_sampson_model", g_use_sampson_model, g_use_sampson_model);
            n.param("use_stereo_correction", g_use_stereo_correction, g_use_stereo_correction);
            n.param("opt_verbose", g_opt_verbose, g_opt_verbose); 
            ROS_INFO("parameters.cpp: gravity_norm: %lf", nG);
            std::cout <<"parameters.cpp: "<< (g_use_sampson_model?"Yes use sampson model":"Not use sampson model")<<std::endl;
            std::cout <<"parameters.cpp: "<< (g_use_stereo_correction?"Yes use geometric correction":"Not use geometric correction")<<std::endl;

            // right cam to imu
            cv::Mat cv_TT;
            fsSettings["body_T_event1"] >> cv_TT;
            Eigen::Matrix4d T;
            cv::cv2eigen(cv_TT, T);
            RIC.push_back(T.block<3, 3>(0, 0));
            TIC.push_back(T.block<3, 1>(0, 3));
            ROS_INFO_STREAM("Extrinsic_R2_event : " << std::endl << RIC[2]);
            ROS_INFO_STREAM("Extrinsic_T2_event : " << std::endl << TIC[2].transpose());

            // transformation between stereo cams 
            {
                cv::Mat cv_R, cv_T; 
                fsSettings["Rrl_event"] >> cv_R; 
                fsSettings["Trl_event"] >> cv_T; 
                cv::cv2eigen(cv_R, Rrl); 
                cv::cv2eigen(cv_T, Trl); 
                Eigen::Quaterniond qq(Rrl); 
                Rrl = qq.normalized();
                Rlr = Rrl.transpose(); 
                Tlr = - Rlr * Trl;  
                ROS_INFO_STREAM("Rrl_event: " << std::endl << Rrl); 
                ROS_INFO_STREAM("Trl_event: " << std::endl << Trl.transpose());  
            }

            //cam para
            {
                std::string ESVIO_FOLDER_PATH = readParam<std::string>(n, "esvio_folder");

                std::string cam_left_Calibfile, cam_right_Calibfile; 
                fsSettings["event_left_calib"] >> cam_left_Calibfile; 
                fsSettings["event_right_calib"] >> cam_right_Calibfile; 

                std::string cam_left_Path = ESVIO_FOLDER_PATH + cam_left_Calibfile; 
                std::string cam_right_Path = ESVIO_FOLDER_PATH + cam_right_Calibfile; 

                ROS_DEBUG("cam_left_Path: %s", cam_left_Path.c_str()); 
                ROS_DEBUG("cam_right_Path: %s", cam_right_Path.c_str());
                CAM_NAMES.push_back(cam_left_Path); 
                CAM_NAMES.push_back(cam_right_Path); 
            }

            //right event to imu
            cv::Mat cv_TT_event;
            fsSettings["body_T_event1"] >> cv_TT_event;
            Eigen::Matrix4d T_event;
            cv::cv2eigen(cv_TT_event, T_event);
            RIC.push_back(T_event.block<3, 3>(0, 0));
            TIC.push_back(T_event.block<3, 1>(0, 3));
            ROS_INFO_STREAM("Extrinsic_R2_event : " << std::endl << RIC[3]);
            ROS_INFO_STREAM("Extrinsic_T2_event : " << std::endl << TIC[3].transpose());

            // transformation between stereo events 
            {
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
            }

            // get event para
            {
                std::string ESVIO_FOLDER_PATH = readParam<std::string>(n, "esvio_folder");

                std::string event_left_Calibfile, event_right_Calibfile; 
                fsSettings["event_left_calib"] >> event_left_Calibfile; 
                fsSettings["event_right_calib"] >> event_right_Calibfile; 

                std::string event_left_Path = ESVIO_FOLDER_PATH + event_left_Calibfile; 
                std::string event_right_Path = ESVIO_FOLDER_PATH + event_right_Calibfile; 

                ROS_DEBUG("event_left_Path: %s", event_left_Path.c_str()); 
                ROS_DEBUG("event_right_Path: %s", event_right_Path.c_str());
                CAM_NAMES.push_back(event_left_Path); 
                CAM_NAMES.push_back(event_right_Path); 
            }

    }


    INIT_DEPTH = -1.0; 
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = fsSettings["rolling_shutter_tr"];
        ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << TR);
    }
    else
    {
        TR = 0;
    }
    
    fsSettings.release();
}

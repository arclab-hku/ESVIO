#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <cmath>

#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <eigen3/Eigen/Geometry>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"
#include "tic_toc.h"

#include "dvs_msgs/Event.h"
#include "dvs_msgs/EventArray.h"
#include "event_detector/event_detector.h"
#include "utility/visualization.h"

#include <opencv2/features2d.hpp>

using namespace std;
using namespace camodocal;
using namespace Eigen;

using Motion_correction_value=std::pair<bool, std::pair<std::pair<Eigen::Vector4d, Eigen::Vector3f>, std::pair< Eigen::Vector2d,std::pair<Eigen::Vector3f,Eigen::Vector3f>>>>;


bool inBorder(const cv::Point2f &pt);
bool inBorder_event(const cv::Point2f &pt);

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);


class FeatureTracker
{
  public:
    FeatureTracker();

    void trackImage(double _cur_time, const cv::Mat &img_left, const cv::Mat &img_right); //stereo feature tracking
    void trackEvent(double _cur_time, const dvs_msgs::EventArray &event_left, const dvs_msgs::EventArray &event_right);
    void trackEvent(double _cur_time, const dvs_msgs::EventArray &event_left, const dvs_msgs::EventArray &event_right, const Motion_correction_value measurements);
    
    void Image_setMask();
    void Event_setMask();

    void addPoints();
    bool updateID(unsigned int i);
    void stereo_readIntrinsicParameter(vector<string> &calib_file);

    void rejectWithF();
    void rejectWithF_event();

    void undistortedPoints();

    cv::Mat getTrackImage();
    cv::Mat getLoopImage();//for loop detection image
    cv::Mat getTrackImage_two();
    cv::Mat getTrackImage_two_point();
    cv::Mat gettimesurface();
    cv::Mat getEventloop();

    double distance(cv::Point2f &pt1, cv::Point2f &pt2);

        // some draw functions
    void event_drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                                   vector<int> &curLeftIds,
                                   vector<cv::Point2f> &curLeftPts, 
                                   vector<cv::Point2f> &curRightPts,
                                   map<int, cv::Point2f> &prevLeftPtsMap);

    void stereo_event_drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                                   vector<int> &curLeftIds,
                                   vector<cv::Point2f> &curLeftPts, 
                                   vector<cv::Point2f> &curRightPts,
                                   map<int, cv::Point2f> &prevLeftPtsMap);

    void stereo_event_loop(const cv::Mat &imLeft);
    
    void event_drawTrack_two(const cv::Mat &imLeft, const cv::Mat &imRight, 
                                vector<int> &curLeftIds,
                                vector<cv::Point2f> &curLeftPts, 
                                vector<cv::Point2f> &curRightPts,
                                map<int, cv::Point2f> &prevLeftPtsMap);

    void event_drawTrack_stereo(const cv::Mat &imLeft, const cv::Mat &imRight, const cv::Mat &PrevimLeft,
                               vector<int> &curLeftIds,
                               vector<int> &curRightIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &curLeftPtsMap,
                               map<int, cv::Point2f> &curRightPtsMap);

    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                            vector<int> &curLeftIds,
                            vector<cv::Point2f> &curLeftPts, 
                            vector<cv::Point2f> &curRightPts,
                            map<int, cv::Point2f> &prevLeftPtsMap);

   
    vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);//caculate the undistorted points
    cv::Point2f undistortedPts(cv::Point2f &pts, camodocal::CameraPtr cam);
  
    vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                    map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts);//caculate the velocity of point
    cv::Mat mask_image;
    cv::Mat mask_event;//mask for event
    cv::Mat fisheye_mask;
    cv::Mat prev_event_mat_left;//event stream of previous frame
    cv::Mat cur_img_left;
    cv::Mat cur_img_right;
    cv::Mat prev_img_left;
    cv::Mat prev_img_right;
    vector<cv::Point2f> n_pts;
    vector<cv::Point2f> n_pts_right;
    vector<cv::Point2f> prev_pts, cur_pts;//previous features
    vector<cv::Point2f> prev_right_pts, cur_right_pts;
    vector<cv::Point2f> forw_pts;//current features
    vector<cv::Point2f> forw_right_pts;
    vector<cv::Point2f> prev_un_pts, cur_un_pts;
    vector<cv::Point2f> prev_un_right_pts, cur_un_right_pts;//right camera undistorted points
    vector<cv::Point2f> pts_velocity;//left camera feature points velocity
    vector<cv::Point2f> right_pts_velocity;//right camera feature points velocity
    vector<int> ids, ids_right;
    vector<int> track_cnt;//total track num
    vector<int> track_cnt_right;//track num between left and right
    map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map;
    map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;//for right cameras
    camodocal::CameraPtr m_camera;
    vector<camodocal::CameraPtr> stereo_m_camera;//stereo use

    double cur_time;//current time
    double prev_time;

    double t_SAE_time = 0;//for time analysis
    int t_SAE_count = 0;

    double t_temporal_time = 0;//for time analysis
    int t_temporal_count = 0;

    double t_spatial_time = 0;//for time analysis
    int t_spatial_count = 0;

    // some visulization parameters
    cv::Mat imTrack;
    cv::Mat imTrack_loop;
    cv::Mat imTrack_two;
    cv::Mat time_surface_visualization_left;
    cv::Mat time_surface_visualization_right;
    cv::Mat event_frame_visualization;
    cv::Mat imTrack_two_point;
    cv::Mat Image_loop;

    map<int, cv::Point2f> prevLeftPtsMap;//previous left features (id with point)
    map<int, cv::Point2f> curLeftPtsMap;//current left features (id with point)
    map<int, cv::Point2f> curRightPtsMap;//

    bool FLAG_DETECTOR_NOSTART=true;
    bool FLAG_DETECTOR_NOSTART_left=true;//init detector
    bool FLAG_DETECTOR_NOSTART_right=true;//

    static int n_id;//left id
    static int n_id_right;//right id


    cv::Mat undist_map1_, undist_map2_ , K_;//


//save image function
    void save_event_tracking(const cv::Mat &image, const double t){
        std::stringstream string_timestamp;
        string_timestamp << std::setprecision(15) << t;
        std::string str_timestamp = string_timestamp.str(); 

        ostringstream path;
        path <<  "/home/cpy/ESVIO/Event_tracking/"
                <<"timestamp:"<< str_timestamp << "---"
                << "tracking.jpg";
        cv::imwrite( path.str().c_str(), image);
    }


    void save_image_tracking(const cv::Mat &image, const double t){
        std::stringstream string_timestamp;
        string_timestamp << std::setprecision(15) << t;
        std::string str_timestamp = string_timestamp.str(); 

        ostringstream path;
        path <<  "/home/cpy/ESVIO/Image_tracking/"
                <<"timestamp:"<< str_timestamp << "---"
                << "tracking.jpg";
        cv::imwrite( path.str().c_str(), image);
    }    
};


template <typename T>
using Mat3 = typename Eigen::Matrix<T, 3, 3>;

/**
 * @brief convert a vector to skew matrix
 *
 * @tparam T
 * @param v
 * @return Mat3<typename T::Scalar>
 */
template <typename T>
Mat3<typename T::Scalar> vectorToSkewMat(const Eigen::MatrixBase<T> &v) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                "Must have 3x1 matrix");
  Mat3<typename T::Scalar> m;
  m << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
  return m;
}

/**
 * @brief convert a vector to homogeneous coordinates
 *
 * @param v
 */
inline void ConvertToHomogeneous(Eigen::Vector3f *v) {
  (*v)[0] = (*v)[0] / (*v)[2];
  (*v)[1] = (*v)[1] / (*v)[2];
  (*v)[2] = 1;
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}

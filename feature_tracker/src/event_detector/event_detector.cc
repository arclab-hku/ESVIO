#include "event_detector.h"
#include <ros/ros.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <eigen3/Eigen/Geometry>

std::mutex events_mutex_;

namespace esvio  { 

EventDetector::EventDetector() :
    kSmallCircle_{{0, 3}, {1, 3}, {2, 2}, {3, 1},
              {3, 0}, {3, -1}, {2, -2}, {1, -3},
              {0, -3}, {-1, -3}, {-2, -2}, {-3, -1},
              {-3, 0}, {-3, 1}, {-2, 2}, {-1, 3}},
    kLargeCircle_{{0, 4}, {1, 4}, {2, 3}, {3, 2},
              {4, 1}, {4, 0}, {4, -1}, {3, -2},
              {2, -3}, {1, -4}, {0, -4}, {-1, -4},
              {-2, -3}, {-3, -2}, {-4, -1}, {-4, 0},
              {-4, 1}, {-3, 2}, {-2, 3}, {-1, 4}} {
}

EventDetector::EventDetector(int col, int row):
    kSmallCircle_{{0, 3}, {1, 3}, {2, 2}, {3, 1},
              {3, 0}, {3, -1}, {2, -2}, {1, -3},
              {0, -3}, {-1, -3}, {-2, -2}, {-3, -1},
              {-3, 0}, {-3, 1}, {-2, 2}, {-1, 3}},
    kLargeCircle_{{0, 4}, {1, 4}, {2, 3}, {3, 2},
              {4, 1}, {4, 0}, {4, -1}, {3, -2},
              {2, -3}, {1, -4}, {0, -4}, {-1, -4},
              {-2, -3}, {-3, -2}, {-4, -1}, {-4, 0},
              {-4, 1}, {-3, 2}, {-2, 3}, {-1, 4}},
              kSensorWidth_{col},
              kSensorHeight_{row}{

    sae_[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_latest_[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_latest_[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
}

EventDetector::~EventDetector() {
}

void EventDetector::init(int col, int row){

    kSensorWidth_=col;
    kSensorHeight_=row;
    sensor_size_ = cv::Size(kSensorWidth_, kSensorHeight_);
    sae_[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_latest_[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_latest_[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_left[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_left[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_right[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_right[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_latest_left[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_latest_left[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_latest_right[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_latest_right[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);

    decay_ms_=para_decay_ms;
    ignore_polarity_= (bool) para_ignore_polarity;
    median_blur_kernel_size_=para_median_blur_kernel_size;
    filter_threshold_=para_feature_filter_threshold;

}

void EventDetector::init(int col, int row, const double fx, const double fy, const double cx, const double cy){

    kSensorWidth_=col;
    kSensorHeight_=row;
    sensor_size_ = cv::Size(kSensorWidth_, kSensorHeight_);
    sae_[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_latest_[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_latest_[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_left[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_left[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_right[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_right[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_latest_left[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_latest_left[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_latest_right[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
    sae_latest_right[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);

    decay_ms_=para_decay_ms;
    ignore_polarity_= (bool) para_ignore_polarity;
    median_blur_kernel_size_=para_median_blur_kernel_size;
    filter_threshold_=para_feature_filter_threshold;
    
    intrinsics_matrix << fx, 0.,         cx,
                         0.,         fy, cy,
                         0.,         0.,          1.;

    cout << intrinsics_matrix << endl;
}

void EventDetector::createSAE_left(double et, int ex, int ey, bool ep, const Motion_correction_value measurements){

      // add motion compensation
      const double const_et=et;
      const double const_ex=ex;
      const double const_ey=ey;

      Eigen::Vector4d State = measurements.second.first.first;//current velocity with time
      Eigen::Vector3f temp_v_pre = measurements.second.first.second;//previous state velocity
      Eigen::Vector3f temp_v;//current velocity
      temp_v[0]=State[0];
      temp_v[1]=State[1];
      temp_v[2]=State[2];

      double temp_time=State[3];

      Eigen::Vector3f accel_avg_=measurements.second.second.second.first;//front end acceleration
      Eigen::Vector3f omega_avg_=measurements.second.second.second.second;//IMU angular velocity

      double t_0 = measurements.second.second.first[0];//time of first event
      double t_1 = measurements.second.second.first[1];//time of current event
      const double dt = et - t_0;
      
      if( sqrt(pow(accel_avg_[0], 2) + pow(accel_avg_[1], 2) + pow(accel_avg_[2], 2)) > a_motion_compensation_threshold){// only when the acceleration is larger than the threshold, do the motion compensation
        Eigen::Vector2d correct_coordinate = motioncorrection(const_ex,const_ey,temp_v,temp_v_pre,accel_avg_,omega_avg_,dt);

        ex=correct_coordinate[0];
        ey=correct_coordinate[1];
      }     

      // Update Surface of Active Events
      const int pol = ep ? 1 : 0;//positive polarity is 1, negative polarity is 0
      const int pol_inv = (!ep) ? 1 : 0;
      double & t_last = sae_latest_[pol](ex,ey);//save time
      double & t_last_inv = sae_latest_[pol_inv](ex, ey);

      if ((et > t_last + filter_threshold_) || (t_last_inv > t_last) ) {
        t_last = et;
        sae_[pol](ex, ey) = et;
      } else {
        t_last = et;
      }

      cur_event_mat_left.at<cv::Vec3b>(cv::Point(ex,ey)) = (
            ep == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));//posivity is blue, negativity is red
}

void EventDetector::createSAE_left(double et, int ex, int ey, bool ep){

      // Update Surface of Active Events 
      const int pol = ep ? 1 : 0;
      const int pol_inv = (!ep) ? 1 : 0;
      double & t_last = sae_latest_[pol](ex,ey);
      double & t_last_inv = sae_latest_[pol_inv](ex, ey);

      if ((et > t_last + filter_threshold_) || (t_last_inv > t_last) ) {
        t_last = et;
        sae_[pol](ex, ey) = et;
      } else {
        t_last = et;
      }

      cur_event_mat_left.at<cv::Vec3b>(cv::Point(ex,ey)) = (
            ep == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
}

void EventDetector::createSAE_right(double et_right, int ex_right, int ey_right, bool ep_right, const Motion_correction_value measurements){

      const double const_et_right=et_right;
      const double const_ex_right=ex_right;
      const double const_ey_right=ey_right;

      Eigen::Vector4d State = measurements.second.first.first;
      Eigen::Vector3f temp_v_pre = measurements.second.first.second;
      Eigen::Vector3f temp_v;
      temp_v[0]=State[0];
      temp_v[1]=State[1];
      temp_v[2]=State[2];
      double temp_time=State[3];

      Eigen::Vector3f accel_avg_=measurements.second.second.second.first;
      Eigen::Vector3f omega_avg_=measurements.second.second.second.second;

      double t_0 = measurements.second.second.first[0];
      double t_1 = measurements.second.second.first[1];
      const double dt = et_right - t_0;

      if( sqrt(pow(accel_avg_[0], 2) + pow(accel_avg_[1], 2) + pow(accel_avg_[2], 2)) > a_motion_compensation_threshold){
        Eigen::Vector2d correct_coordinate = motioncorrection(const_ex_right,const_ey_right,temp_v, temp_v_pre, accel_avg_,omega_avg_,dt);

        ex_right=correct_coordinate[0];
        ey_right=correct_coordinate[1];
      }

      const int pol_right = ep_right ? 1 : 0;
      const int pol_inv_right = (!ep_right) ? 1 : 0;
      double & t_last_right = sae_latest_right[pol_right](ex_right,ey_right);
      double & t_last_inv_right = sae_latest_right[pol_inv_right](ex_right, ey_right);

      if ((et_right > t_last_right + filter_threshold_) || (t_last_inv_right > t_last_right) ) {
        t_last_right = et_right;
        sae_right[pol_right](ex_right, ey_right) = et_right;
      } else {
        t_last_right = et_right;
      }

      cur_event_mat_right.at<cv::Vec3b>(cv::Point(ex_right, ey_right)) = (
            ep_right == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
}

void EventDetector::createSAE_right(double et_right, int ex_right, int ey_right, bool ep_right){

      const int pol_right = ep_right ? 1 : 0;
      const int pol_inv_right = (!ep_right) ? 1 : 0;
      double & t_last_right = sae_latest_right[pol_right](ex_right,ey_right);
      double & t_last_inv_right = sae_latest_right[pol_inv_right](ex_right, ey_right);

      if ((et_right > t_last_right + filter_threshold_) || (t_last_inv_right > t_last_right) ) {
        t_last_right = et_right;
        sae_right[pol_right](ex_right, ey_right) = et_right;
      } else {
        t_last_right = et_right;
      }

      cur_event_mat_right.at<cv::Vec3b>(cv::Point(ex_right, ey_right)) = (
            ep_right == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
}

cv::Mat EventDetector::SAEtoTimeSurface_left( const double external_sync_time){

    // create exponential-decayed Time Surface map
    const double decay_sec = decay_ms_ / 1000.0;//ms to s
    cv::Mat time_surface_map;
    time_surface_map=cv::Mat::zeros(sensor_size_, CV_64F);

      for (int y=0;y<sensor_size_.height;++y){
              for(int x=0;x<sensor_size_.width;++x){

                    double most_recent_stamp_at_coordXY= (sae_[1](x,y)>sae_[0](x,y)) ? sae_[1](x,y) : sae_[0](x,y);

                    if(most_recent_stamp_at_coordXY > 0){
                          const double dt = (external_sync_time-most_recent_stamp_at_coordXY);
                          double expVal =std::exp(-dt / decay_sec);

                          if(!ignore_polarity_)
                            {
                                double polarity = (sae_[1](x,y)>sae_[0](x,y)) ? 1.0 : -1.0;  
                                expVal *= polarity;
                            }
                            time_surface_map.at<double>(y,x) = expVal;
                    }
              }
      }

      if(!ignore_polarity_)
        time_surface_map = 255.0 * (time_surface_map + 1.0) / 2.0;
      else
        time_surface_map = 255.0 * time_surface_map;
      time_surface_map.convertTo(time_surface_map, CV_8U);

      if(median_blur_kernel_size_ > 0){
          cv::medianBlur(time_surface_map, time_surface_map, 2 * median_blur_kernel_size_ + 1);
      }

    return time_surface_map;
}

cv::Mat EventDetector::SAEtoTimeSurface_right( const double external_sync_time){

    const double decay_sec = decay_ms_ / 1000.0;
    cv::Mat time_surface_map_right;
    time_surface_map_right=cv::Mat::zeros(sensor_size_, CV_64F);

      for (int y=0;y<sensor_size_.height;++y){
              for(int x=0;x<sensor_size_.width;++x){

                    double most_recent_stamp_at_coordXY= (sae_right[1](x,y)>sae_right[0](x,y)) ? sae_right[1](x,y) : sae_right[0](x,y);

                    if(most_recent_stamp_at_coordXY > 0){
                          const double dt = (external_sync_time-most_recent_stamp_at_coordXY);
                          double expVal =std::exp(-dt / decay_sec);

                          if(!ignore_polarity_)
                            {
                                double polarity = (sae_right[1](x,y)>sae_right[0](x,y)) ? 1.0 : -1.0;  
                                expVal *= polarity;
                            }
                            time_surface_map_right.at<double>(y,x) = expVal;
                    }
              }
      }

      if(!ignore_polarity_)
        time_surface_map_right = 255.0 * (time_surface_map_right + 1.0) / 2.0;
      else
        time_surface_map_right = 255.0 * time_surface_map_right;
      time_surface_map_right.convertTo(time_surface_map_right, CV_8U);

      if(median_blur_kernel_size_ > 0){
          cv::medianBlur(time_surface_map_right, time_surface_map_right, 2 * median_blur_kernel_size_ + 1);
      }

    return time_surface_map_right;
}


bool EventDetector::isCorner(double et, int ex, int ey, bool ep) {

    const int pol = ep ? 1 : 0;
    const int pol_inv = (!ep) ? 1 : 0;
    double & t_last = sae_latest_[pol](ex,ey);
    double & t_last_inv = sae_latest_[pol_inv](ex, ey);

    if ((et > t_last + filter_threshold_) || (t_last_inv > t_last) ) {
      return false;
    }

    // Return if too close to the border
    const int kBorderLimit = MIN_DIST + 1;
    if (ex < kBorderLimit || ex >= (kSensorWidth_ - kBorderLimit) ||
        ey < kBorderLimit || ey >= (kSensorHeight_ - kBorderLimit)) {
      return false;
    }

    // Define constant and thresholds
    const int kSmallCircleSize = 16;
    const int kLargeCircleSize = 20;
    const int kSmallMinThresh = 4;
    const int kSmallMaxThresh = 6;
    const int kLargeMinThresh = 5;
    const int kLargeMaxThresh = 8;


    bool is_arc_valid = false;

    // Small Circle exploration
    // Initialize arc from newest element
    double segment_new_min_t = sae_[pol](ex+kSmallCircle_[0][0], ey+kSmallCircle_[0][1]);

    // Left and Right are equivalent to CW and CCW as in the paper
    int arc_right_idx = 0;
    int arc_left_idx;

    // Find newest
    for (int i=1; i<kSmallCircleSize; i++) {
      const double t =sae_[pol](ex+kSmallCircle_[i][0], ey+kSmallCircle_[i][1]);
      if (t > segment_new_min_t) {
        segment_new_min_t = t;
        arc_right_idx = i; 
      }
    }
    // Shift to the sides of the newest element;
    arc_left_idx = (arc_right_idx-1+kSmallCircleSize)%kSmallCircleSize;
    arc_right_idx= (arc_right_idx+1)%kSmallCircleSize;
    double arc_left_value = sae_[pol](ex+kSmallCircle_[arc_left_idx][0], ey+kSmallCircle_[arc_left_idx][1]);
    double arc_right_value = sae_[pol](ex+kSmallCircle_[arc_right_idx][0], ey+kSmallCircle_[arc_right_idx][1]);
    double arc_left_min_t = arc_left_value;
    double arc_right_min_t = arc_right_value;

    // Expand
    // Initial expand does not require checking
    int iteration = 1; // The arc already contain the maximum
    for (; iteration<kSmallMinThresh; iteration++) {
      // Decide the most promising arc
      if (arc_right_value > arc_left_value) { // Right arc
        if (arc_right_min_t < segment_new_min_t) {
            segment_new_min_t = arc_right_min_t;
        }
        // Expand arc
        arc_right_idx= (arc_right_idx+1)%kSmallCircleSize;
        arc_right_value = sae_[pol](ex+kSmallCircle_[arc_right_idx][0], ey+kSmallCircle_[arc_right_idx][1]);
        if (arc_right_value < arc_right_min_t) { // Update minimum of the arc
          arc_right_min_t = arc_right_value;
        }
      } else { // Left arc
        // Include arc in new segment
        if (arc_left_min_t < segment_new_min_t) {
          segment_new_min_t = arc_left_min_t;
        }

        // Expand arc
        arc_left_idx= (arc_left_idx-1+kSmallCircleSize)%kSmallCircleSize;
        arc_left_value = sae_[pol](ex+kSmallCircle_[arc_left_idx][0], ey+kSmallCircle_[arc_left_idx][1]);
        if (arc_left_value < arc_left_min_t) { // Update minimum of the arc
          arc_left_min_t = arc_left_value;
        }
      }
    }
    int newest_segment_size = kSmallMinThresh;

    // Further expand until completion of the circle
    for (; iteration<kSmallCircleSize; iteration++) {
      // Decide the most promising arc
      if (arc_right_value > arc_left_value) { // Right arc
        // Include arc in new segment
        if ((arc_right_value >=  segment_new_min_t)) {
          newest_segment_size = iteration+1; // Check
          if (arc_right_min_t < segment_new_min_t) {
            segment_new_min_t = arc_right_min_t;
          }
        }

        // Expand arc
        arc_right_idx= (arc_right_idx+1)%kSmallCircleSize;
        arc_right_value = sae_[pol](ex+kSmallCircle_[arc_right_idx][0], ey+kSmallCircle_[arc_right_idx][1]);
        if (arc_right_value < arc_right_min_t) { // Update minimum of the arc
          arc_right_min_t = arc_right_value;
        }
      } else { // Left arc
        // Include arc in new segment
        if ((arc_left_value >=  segment_new_min_t)) {
          newest_segment_size = iteration+1;
          if (arc_left_min_t < segment_new_min_t) {
            segment_new_min_t = arc_left_min_t;
          }
        }

        // Expand arc
        arc_left_idx= (arc_left_idx-1+kSmallCircleSize)%kSmallCircleSize;
        arc_left_value = sae_[pol](ex+kSmallCircle_[arc_left_idx][0], ey+kSmallCircle_[arc_left_idx][1]);
        if (arc_left_value < arc_left_min_t) { // Update minimum of the arc
          arc_left_min_t = arc_left_value;
        }
      }
    }

    if (// Corners with newest segment of a minority of elements in the circle
        // These corners are equivalent to those in Mueggler et al. BMVC17
            (newest_segment_size <= kSmallMaxThresh) ||
        // Corners with newest segment of a majority of elements in the circle
        // This can be commented out to decrease noise at expenses of less repeatibility. If you do, DO NOT forget to comment the equilvent line in the large circle
        ((newest_segment_size >= (kSmallCircleSize - kSmallMaxThresh)) && (newest_segment_size <= (kSmallCircleSize - kSmallMinThresh)))) {
      is_arc_valid = true;
    }

    // Large Circle exploration
    if (is_arc_valid) {
    is_arc_valid = false;

      segment_new_min_t = sae_[pol](ex+kLargeCircle_[0][0], ey+kLargeCircle_[0][1]);
      arc_right_idx = 0;

      // Initialize in the newest element
      for (int i=1; i<kLargeCircleSize; i++) {
        const double t =sae_[pol](ex+kLargeCircle_[i][0], ey+kLargeCircle_[i][1]);
        if (t > segment_new_min_t) {
          segment_new_min_t = t;
          arc_right_idx = i; // % End up in the maximum value
        }
      }
      // Shift to the sides of the newest elements;
      arc_left_idx = (arc_right_idx-1+kLargeCircleSize)%kLargeCircleSize;
      arc_right_idx= (arc_right_idx+1)%kLargeCircleSize;
      arc_left_value = sae_[pol](ex+kLargeCircle_[arc_left_idx][0],
                                 ey+kLargeCircle_[arc_left_idx][1]);
      arc_right_value = sae_[pol](ex+kLargeCircle_[arc_right_idx][0],
                                  ey+kLargeCircle_[arc_right_idx][1]);
      arc_left_min_t = arc_left_value;
      arc_right_min_t = arc_right_value;

      // Expand
      // Initial expand does not require checking
      iteration = 1;
      for (; iteration<kLargeMinThresh; iteration++) {
        // Decide the most promising arc
        if (arc_right_value > arc_left_value) { // Right arc
          if (arc_right_min_t < segment_new_min_t) {
              segment_new_min_t = arc_right_min_t;
          }
          // Expand arc
          arc_right_idx= (arc_right_idx+1)%kLargeCircleSize;
          arc_right_value = sae_[pol](ex+kLargeCircle_[arc_right_idx][0],
                                      ey+kLargeCircle_[arc_right_idx][1]);
          if (arc_right_value < arc_right_min_t) { // Update minimum of the arc
            arc_right_min_t = arc_right_value;
          }
        } else { // Left arc
          // Include arc in new segment
          if (arc_left_min_t < segment_new_min_t) {
            segment_new_min_t = arc_left_min_t;
          }

          // Expand arc
          arc_left_idx= (arc_left_idx-1+kLargeCircleSize)%kLargeCircleSize;
          arc_left_value = sae_[pol](ex+kLargeCircle_[arc_left_idx][0],
                                     ey+kLargeCircle_[arc_left_idx][1]);
          if (arc_left_value < arc_left_min_t) { // Update minimum of the arc
            arc_left_min_t = arc_left_value;
          }
        }
      }
      newest_segment_size = kLargeMinThresh;

      // Further expand until completion of the circle
      for (; iteration<kLargeCircleSize; iteration++) {
        // Decide the most promising arc
        if (arc_right_value > arc_left_value) { // Right arc
          // Include arc in new segment
          if ((arc_right_value >=  segment_new_min_t)) {
            newest_segment_size = iteration+1;
            if (arc_right_min_t < segment_new_min_t) {
              segment_new_min_t = arc_right_min_t;
            }
          }

          // Expand arc
          arc_right_idx= (arc_right_idx+1)%kLargeCircleSize;
          arc_right_value = sae_[pol](ex+kLargeCircle_[arc_right_idx][0],
                                      ey+kLargeCircle_[arc_right_idx][1]);
          if (arc_right_value < arc_right_min_t) { // Update minimum of the arc
            arc_right_min_t = arc_right_value;
          }
        } else { // Left arc
          // Include arc in new segment
          if ((arc_left_value >=  segment_new_min_t)) {
            newest_segment_size = iteration+1;
            if (arc_left_min_t < segment_new_min_t) {
              segment_new_min_t = arc_left_min_t;
            }
          }

          // Expand arc
          arc_left_idx= (arc_left_idx-1+kLargeCircleSize)%kLargeCircleSize;
          arc_left_value = sae_[pol](ex+kLargeCircle_[arc_left_idx][0],
                                    ey+kLargeCircle_[arc_left_idx][1]);
          if (arc_left_value < arc_left_min_t) { // Update minimum of the arc
            arc_left_min_t = arc_left_value;
          }
        }
      }

      if (// Corners with newest segment of a minority of elements in the circle
          // These corners are equivalent to those in Mueggler et al. BMVC17
              (newest_segment_size <= kLargeMaxThresh) ||
          // Corners with newest segment of a majority of elements in the circle
          // This can be commented out to decrease noise at expenses of less repeatibility. If you do, DO NOT forget to comment the equilvent line in the small circle
          (newest_segment_size >= (kLargeCircleSize - kLargeMaxThresh) && (newest_segment_size <= (kLargeCircleSize - kLargeMinThresh))) ) {
        is_arc_valid = true;
      }
    }

    return is_arc_valid;
}


Eigen::Vector2d EventDetector::motioncorrection(const double ex,const double ey,const Eigen::Vector3f tmp_v,const Eigen::Vector3f tmp_v_pre,const Eigen::Vector3f accel_avg_, const Eigen::Vector3f omega_avg_,const double dt)
{
  Eigen::Vector2d correct_coordinate;
  const int kBorder = 6;

  if(ex > kBorder && ex <= (kSensorWidth_ - kBorder) &&
      ey > kBorder && ey <= (kSensorHeight_ - kBorder) ){//only process non-border points

      Eigen::Vector3f rotation_vector = omega_avg_ * dt;
      Eigen::Matrix3f rot_skew_mat = vectorToSkewMat(rotation_vector);
      Eigen::Matrix3f rotation_matrix_ = rot_skew_mat.exp();
      Eigen::Matrix3f rot_K =intrinsics_matrix *rotation_matrix_.transpose()* intrinsics_matrix.inverse();
      Eigen::Vector3f trans_K = 0.5 * dt * (tmp_v + tmp_v_pre);

      trans_K=-rot_K*(intrinsics_matrix.inverse()*trans_K);      

      //points before correction
      Eigen::Vector3f eventVec;
      eventVec[0] = ex;
      eventVec[1] = ey;
      eventVec[2] = 1;
      eventVec = rot_K * eventVec+trans_K;  // event warp to t_0
      ConvertToHomogeneous(&eventVec);//homogeneous

      int x_coordinate = std::floor(eventVec[0]);
      int y_coordinate = std::floor(eventVec[1]);

      if(x_coordinate>0 && x_coordinate< kSensorWidth_ -1&& y_coordinate >0 && y_coordinate <kSensorHeight_-1)//avoid out of range
      {
        correct_coordinate[0]=x_coordinate;
        correct_coordinate[1]=y_coordinate;
      }    
      else{
        correct_coordinate[0] = ex;
        correct_coordinate[1] = ey;
      }
  
  }
  else{
    correct_coordinate[0] = ex;
    correct_coordinate[1] = ey;
  }

  return correct_coordinate;
}


}

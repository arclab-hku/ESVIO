#ifndef EVENT_DETECTOR_H
#define EVENT_DETECTOR_H

#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../parameters.h"
#include <cmath>
#include <thread>
#include "../feature_tracker.h"
#include <mutex>

namespace esvio { // Asynchronous Corner Detector

using Motion_correction_value=std::pair<bool, std::pair<std::pair<Eigen::Vector4d, Eigen::Vector3f>, std::pair< Eigen::Vector2d,std::pair<Eigen::Vector3f,Eigen::Vector3f>>>>;

class EventDetector
{
public:
  EventDetector();
  EventDetector(int col, int row);
  ~EventDetector();

void init(int col, int row);
void init(int col, int row, const double fx, const double fy, const double cx, const double cy);
void createSAE_left(double et, int ex, int ey, bool ep);
void createSAE_right(double et_right, int ex_right, int ey_right, bool ep_right);
void createSAE_left(double et, int ex, int ey, bool ep, const Motion_correction_value measurements);
void createSAE_right(double et_right, int ex_right, int ey_right, bool ep_right, const Motion_correction_value measurements);

cv::Mat SAEtoTimeSurface_left(const double external_sync_time);
cv::Mat SAEtoTimeSurface_right(const double external_sync_time);


bool isCorner(double et, int ex, int ey, bool ep);

Eigen::Vector2d motioncorrection(const double ex,const double ey,const Eigen::Vector3f tmp_v,const Eigen::Vector3f tmp_v_pre,const Eigen::Vector3f accel_avg_, const Eigen::Vector3f omega_avg_, const double dt);


double last_event_time;

cv::Mat cur_event_mat_left;
cv::Mat cur_event_mat_right;

dvs_msgs::EventArray motion_correct_eventstream;

private:
  
  const double t_motion_compensation_threshold = 0.01;
  const double a_motion_compensation_threshold = 5;
  int t_motion_count_output = 0;
  int t_motion_count = 0;
  double t_motion_time = 0;


  // Circular Breshenham Masks
  const int kSmallCircle_[16][2];
  const int kLargeCircle_[20][2];

  int kSensorWidth_;
  int kSensorHeight_;
  Eigen::Matrix3f intrinsics_matrix;

  cv::Size sensor_size_;
  double decay_ms_;
  bool ignore_polarity_;
  int median_blur_kernel_size_;
  double decay_ms_for_loop;

  double filter_threshold_;

  // Surface of Active Events
  Eigen::MatrixXd sae_[2];//save sae
  Eigen::MatrixXd sae_latest_[2];//save previous sae
  Eigen::MatrixXd sae_left[2];
  Eigen::MatrixXd sae_latest_left[2];
  Eigen::MatrixXd sae_right[2];
  Eigen::MatrixXd sae_latest_right[2];
};

} // Asynchronous Corner Detector

#endif // EVENT_DETECTOR_H

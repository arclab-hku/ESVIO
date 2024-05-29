#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
using namespace std;

#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <ros/console.h>
#include <ros/assert.h>

#include "parameters.h"

enum DPT_TYPE
{
    NO_DEPTH =0, DEPTH_MES, DEPTH_TRI, INVALID
} ;

class stereo_FeaturePerFrame
{
  public:
    stereo_FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        ori_point = point; 
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5); 
        velocity.y() = _point(6); 
        cur_td = td;
        is_stereo = false;
        dpt = -1; 
        gc_succeed =false; 
    }
    void rightObservation(const Eigen::Matrix<double, 7, 1> &_point)
    {
        pointRight.x() = _point(0);
        pointRight.y() = _point(1);
        pointRight.z() = _point(2);
        ori_pointRight = pointRight; 
        uvRight.x() = _point(3);
        uvRight.y() = _point(4);
        velocityRight.x() = _point(5); 
        velocityRight.y() = _point(6); 
        is_stereo = true;

        // call getDepth()
        getDepth();
    }

    // triangulation to compute depth 
    double getDepth(); 

    void print(){
        printf("point: %f %f %f uv: %f %f \n", point.x(), point.y(), point.z(), uv.x(), uv.y());
    }
    double cur_td;
    bool gc_succeed; 
    Vector3d point, pointRight;
    Vector3d ori_point, ori_pointRight; 
    Vector2d uv, uvRight;
    Vector2d velocity, velocityRight;
    double z;
    bool is_used;
    double parallax;
    MatrixXd A;
    VectorXd b;
    double dep_gradient;
    bool is_stereo; 
    double dpt; 
    int feat_id; 
};

class stereo_FeaturePerId
{
  public:
    const int feature_id;
    int start_frame;
    vector<stereo_FeaturePerFrame> feature_per_frame;

    int used_num;
    bool is_outlier;
    bool is_margin;
    double estimated_depth;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

    Vector3d gt_p;

    stereo_FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(0),dpt_type(NO_DEPTH)
    {
    }

    int endFrame();
    DPT_TYPE dpt_type; 
};


class Stereo_Event_FeaturePerFrame
{
  public:
    Stereo_Event_FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        ori_point = point;//define
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5); 
        velocity.y() = _point(6); 
        cur_td = td;
        is_stereo = false;
        dpt = -1;
        gc_succeed = false;
    }

    void rightObservation(const Eigen::Matrix<double, 7, 1> &_point)
    {
        pointRight.x() = _point(0);
        pointRight.y() = _point(1);
        pointRight.z() = _point(2);
        ori_pointRight = pointRight; 
        uvRight.x() = _point(3);
        uvRight.y() = _point(4);
        velocityRight.x() = _point(5); 
        velocityRight.y() = _point(6); 
        is_stereo = true;

        // call getDepth()
        getDepth();
    }
    //triangulation to compute depth
    double getDepth();

    void print(){
        printf("point: %f %f %f uv: %f %f \n", point.x(), point.y(), point.z(), uv.x(), uv.y());
    }
    double cur_td;
    bool gc_succeed; 
    Vector3d point, pointRight;
    Vector3d ori_point, ori_pointRight; 
    Vector2d uv, uvRight;
    Vector2d velocity, velocityRight;
    double z;
    bool is_used;
    double parallax;
    MatrixXd A;
    VectorXd b;
    double dep_gradient;
    bool is_stereo; 
    double dpt; 
    int feat_id; 
};

class Stereo_Event_FeaturePerId
{
  public:
    const int feature_id;
    int start_frame;
    vector<Stereo_Event_FeaturePerFrame> Event_feature_per_frame;

    int used_num;
    bool is_outlier;
    bool is_margin;
    double estimated_depth;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

    Vector3d gt_p;

    Stereo_Event_FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(0), dpt_type(NO_DEPTH)
    {
    }

    int endFrame();
    DPT_TYPE dpt_type;
};

class FeatureManager
{
  public:
    FeatureManager(Matrix3d _Rs[]);

    void setRic(Matrix3d _ric[]);
    void Event_setRic(Matrix3d _ric[]);

    void clearState();

    int Stereo_getImageFeatureCount();
    int Stereo_getEventFeatureCount();
    
 
    bool stereo_addFeatureCheckParallax( int frame_count, 
                                  const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
                                  const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &event,
                                  double td);
    bool ESIO_addFeatureCheckParallax( int frame_count, 
                                  const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &event, 
                                  double td);

    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);
    vector<pair<Vector3d, Vector3d>> Event_getCorresponding(int frame_count_l, int frame_count_r);

    vector<pair<Vector3d, Vector3d>> getCorrespondingWithDepth(int frame_count_l, int frame_count_r);
    vector<pair<Vector3d, Vector3d>> Event_getCorrespondingWithDepth(int frame_count_l, int frame_count_r);

    void Stereo_Image_setDepth(const VectorXd &x);
    void Stereo_Event_setDepth(const VectorXd &x);//set stereo event depth

    void removeFailures();
    void clearDepth(const VectorXd &x);
    void Event_clearDepth(const VectorXd &x);
    VectorXd stereo_getDepthVector();
    VectorXd Stereo_Event_getDepthVector();
    void triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
    void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d);
    void Stereo_Event_triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d);
    void triangulateStereo();
    void Event_triangulateStereo();
    void stereo_triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);// for points not be trangulated by stereo，use mono trangulation
    void Stereo_Event_triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    void removeBack();
    void removeFront(int frame_count);
    list<stereo_FeaturePerId> stereo_feature;//stereo image feature
    list<Stereo_Event_FeaturePerId> Stereo_Event_feature;//stereo event feature
    int last_track_num;
    double last_average_parallax;
    int new_feature_num;
    int long_track_num;

  private:
    double compensatedParallax2(const stereo_FeaturePerId &it_per_id, int frame_count);
    double compensatedParallax2(const Stereo_Event_FeaturePerId &it_per_id, int frame_count);
    const Matrix3d *Rs;
    Matrix3d ric[NUM_OF_CAM_stereo * 2];
};

#endif
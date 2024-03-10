#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <ros/package.h>
#include <mutex>
#include <queue>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "pose_graph.h"
#include "utility/CameraPoseVisualization.h"
#include "parameters.h"
#include <cmath>
#define SKIP_FIRST_CNT 10
using namespace std;

queue<sensor_msgs::ImageConstPtr> image_buf;//time surface
queue<sensor_msgs::PointCloudConstPtr> point_buf;// pointcloud in the world coordinate
queue<nav_msgs::Odometry::ConstPtr> pose_buf;// current pose
queue<Eigen::Vector3d> odometry_buf;
std::mutex m_buf;
std::mutex m_process;
int frame_index  = 0;
int sequence = 1;
PoseGraph posegraph;//pose graph
int skip_first_cnt = 0;
int SKIP_CNT;
int skip_cnt = 0;
bool load_flag = 0;
bool start_flag = 0;
double SKIP_DIS = 0;

int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
int ROW;
int COL;
int DEBUG_IMAGE;
int SAVE_LOOP_MATCH;
int VISUALIZE_IMU_FORWARD;
int LOOP_CLOSURE;
int FAST_RELOCALIZATION;
double TS_LK_THRESHOLD;
int system_mode = 0;// 0 for ESIO, 1 for ESVIO

camodocal::CameraPtr m_camera;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;
ros::Publisher pub_match_img;
ros::Publisher pub_match_points;
ros::Publisher pub_camera_pose_visual;
ros::Publisher pub_key_odometrys;
ros::Publisher pub_vio_path;
ros::Publisher pub_latest_loop_evio;
ros::Publisher pub_flight_evio;
nav_msgs::Path no_loop_path;

std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
std::string ESVIO_RESULT_PATH;
CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
Eigen::Vector3d last_t(-100, -100, -100);
double last_image_time = -1;

using EventQueue = std::queue<dvs_msgs::EventArray>;
EventQueue events_buf;//event_corner_buf


void new_sequence()//open new sequence, map fusion
{
    printf("new sequence\n");
    sequence++;
    printf("sequence cnt %d \n", sequence);
    if (sequence > 5)
    {
        ROS_WARN("only support 5 sequences since it's boring to copy code for more sequences.");
        ROS_BREAK();
    }
    posegraph.posegraph_visualization->reset();
    posegraph.publish();
    m_buf.lock();
    while(!image_buf.empty())
        image_buf.pop();
    while(!point_buf.empty())
        point_buf.pop();
    while(!pose_buf.empty())
        pose_buf.pop();
    while(!odometry_buf.empty())
        odometry_buf.pop();
    while(!events_buf.empty())//event-corner feature buf
        events_buf.pop();
    m_buf.unlock();
}

void eventsfeatureCallback(const dvs_msgs::EventArray &event_feature_msg){// callback event-corner feature
    m_buf.lock();
    events_buf.push(event_feature_msg);
    m_buf.unlock();
}

void loop_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    image_buf.push(image_msg);
    m_buf.unlock();

    if (last_image_time == -1)
        last_image_time = image_msg->header.stamp.toSec();
    else if (image_msg->header.stamp.toSec() - last_image_time > 1.0 || image_msg->header.stamp.toSec() < last_image_time) // check timestamp
    {
        
        ROS_WARN("the event camera has stoped, waiting for the restart!");

    }
    last_image_time = image_msg->header.stamp.toSec();
}


void point_callback(const sensor_msgs::PointCloudConstPtr &point_msg)
{
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    point_buf.push(point_msg);// push keyframe map points into buf
    m_buf.unlock();
    
}


void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();

}


bool imu_forward_flag=false;

void imu_forward_callback(const nav_msgs::Odometry::ConstPtr &forward_msg)// pub imu-frequence pose
{// imu forward pose, get low-delay and high-frequency pose
    if (VISUALIZE_IMU_FORWARD)
    {
        Vector3d vio_t(forward_msg->pose.pose.position.x, forward_msg->pose.pose.position.y, forward_msg->pose.pose.position.z);
        Quaterniond vio_q;
        vio_q.w() = forward_msg->pose.pose.orientation.w;
        vio_q.x() = forward_msg->pose.pose.orientation.x;
        vio_q.y() = forward_msg->pose.pose.orientation.y;
        vio_q.z() = forward_msg->pose.pose.orientation.z;
        // get the newest result

        vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
        vio_q = posegraph.w_r_vio *  vio_q;

        vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
        vio_q = posegraph.r_drift * vio_q;

        // pub odometry
        nav_msgs::Odometry odometry;
        odometry.header = forward_msg->header;
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = vio_t.x();
        odometry.pose.pose.position.y = vio_t.y();
        odometry.pose.pose.position.z = vio_t.z();
        odometry.pose.pose.orientation.x = vio_q.x();
        odometry.pose.pose.orientation.y = vio_q.y();
        odometry.pose.pose.orientation.z = vio_q.z();
        odometry.pose.pose.orientation.w = vio_q.w();
        odometry.twist.twist.linear.x = forward_msg->twist.twist.linear.x;
        odometry.twist.twist.linear.y = forward_msg->twist.twist.linear.y;
        odometry.twist.twist.linear.z = forward_msg->twist.twist.linear.z;
        pub_flight_evio.publish(odometry);

        Vector3d vio_t_cam;
        Quaterniond vio_q_cam;
        vio_t_cam = vio_t + vio_q * tic;
        vio_q_cam = vio_q * qic;        

        cameraposevisual.reset();
        cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
        cameraposevisual.publish_by(pub_camera_pose_visual, forward_msg->header);

        // define a publisher to publish vio_t_cam and vio_q_cam
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = forward_msg->header;
        pose_stamped.pose.position.x=vio_t_cam.x();
        pose_stamped.pose.position.y=vio_t_cam.y();  
        pose_stamped.pose.position.z=vio_t_cam.z(); 
        pose_stamped.pose.orientation.w=vio_q_cam.w();
        pose_stamped.pose.orientation.x=vio_q_cam.x();  
        pose_stamped.pose.orientation.y=vio_q_cam.y();  
        pose_stamped.pose.orientation.z=vio_q_cam.z();
        
        // set flag
        if(!imu_forward_flag && abs(pose_stamped.pose.position.z)<=1.0 && abs(pose_stamped.pose.position.x)<=1.0 && abs(pose_stamped.pose.position.y)<=1.0){
            imu_forward_flag=true;
            // ROS_INFO("start output");
        }
        if (imu_forward_flag)
            pub_latest_loop_evio.publish(pose_stamped);// pub imu rate odom
    }
}


void relo_relative_pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    Vector3d relative_t = Vector3d(pose_msg->pose.pose.position.x,
                                   pose_msg->pose.pose.position.y,
                                   pose_msg->pose.pose.position.z);
    Quaterniond relative_q;
    relative_q.w() = pose_msg->pose.pose.orientation.w;
    relative_q.x() = pose_msg->pose.pose.orientation.x;
    relative_q.y() = pose_msg->pose.pose.orientation.y;
    relative_q.z() = pose_msg->pose.pose.orientation.z;
    double relative_yaw = pose_msg->twist.twist.linear.x;
    int index = pose_msg->twist.twist.linear.y;
    //printf("receive index %d \n", index );
    Eigen::Matrix<double, 8, 1 > loop_info;
    loop_info << relative_t.x(), relative_t.y(), relative_t.z(),
                 relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                 relative_yaw;
    posegraph.updateKeyFrameLoop(index, loop_info);
}


void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //ROS_INFO("vio_callback!");
    Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;

    vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
    vio_q = posegraph.w_r_vio *  vio_q;

    vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
    vio_q = posegraph.r_drift * vio_q;

    Vector3d vio_t_cam;
    Quaterniond vio_q_cam;
    vio_t_cam = vio_t + vio_q * tic;
    vio_q_cam = vio_q * qic;        

    if (!VISUALIZE_IMU_FORWARD)
    {
        cameraposevisual.reset();
        cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
        cameraposevisual.publish_by(pub_camera_pose_visual, pose_msg->header);
    }

    odometry_buf.push(vio_t_cam);
    if (odometry_buf.size() > 10)
    {
        odometry_buf.pop();
    }

    visualization_msgs::Marker key_odometrys;
    key_odometrys.header = pose_msg->header;
    key_odometrys.header.frame_id = "world";
    key_odometrys.ns = "key_odometrys";
    key_odometrys.type = visualization_msgs::Marker::SPHERE_LIST;
    key_odometrys.action = visualization_msgs::Marker::ADD;
    key_odometrys.pose.orientation.w = 1.0;
    key_odometrys.lifetime = ros::Duration();

    key_odometrys.id = 0; 
    key_odometrys.scale.x = 0.1;
    key_odometrys.scale.y = 0.1;
    key_odometrys.scale.z = 0.1;
    key_odometrys.color.r = 1.0;
    key_odometrys.color.a = 1.0;

    for (unsigned int i = 0; i < odometry_buf.size(); i++)
    {
        geometry_msgs::Point pose_marker;
        Vector3d vio_t;
        vio_t = odometry_buf.front();
        odometry_buf.pop();
        pose_marker.x = vio_t.x();
        pose_marker.y = vio_t.y();
        pose_marker.z = vio_t.z();
        key_odometrys.points.push_back(pose_marker);
        odometry_buf.push(vio_t);
    }
    pub_key_odometrys.publish(key_odometrys);

    if (!LOOP_CLOSURE)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = pose_msg->header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = vio_t.x();
        pose_stamped.pose.position.y = vio_t.y();
        pose_stamped.pose.position.z = vio_t.z();
        no_loop_path.header = pose_msg->header;
        no_loop_path.header.frame_id = "world";
        no_loop_path.poses.push_back(pose_stamped);
        pub_vio_path.publish(no_loop_path);//pub no loop path
    }
}

void extrinsic_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    m_process.lock();
    tic = Vector3d(pose_msg->pose.pose.position.x,
                   pose_msg->pose.pose.position.y,
                   pose_msg->pose.pose.position.z);
    qic = Quaterniond(pose_msg->pose.pose.orientation.w,
                      pose_msg->pose.pose.orientation.x,
                      pose_msg->pose.pose.orientation.y,
                      pose_msg->pose.pose.orientation.z).toRotationMatrix();
    m_process.unlock();
}

void process()
{
    if (!LOOP_CLOSURE)
        return;
    while (true)
    {   
        sensor_msgs::ImageConstPtr image_msg = NULL;
        sensor_msgs::PointCloudConstPtr point_msg = NULL;
        nav_msgs::Odometry::ConstPtr pose_msg = NULL;

        m_buf.lock();

        if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            if (image_buf.front()->header.stamp.toSec() > pose_buf.front()->header.stamp.toSec())
            {
                pose_buf.pop();
                printf("throw pose at beginning\n");
            }
            else if (image_buf.front()->header.stamp.toSec() > point_buf.front()->header.stamp.toSec())
            {
                point_buf.pop();
                printf("throw point at beginning\n");
            }

            else if (image_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec() 
                && point_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec())
            {
                pose_msg = pose_buf.front();
                pose_buf.pop();
                while (!pose_buf.empty()) 
                    pose_buf.pop();
                while (image_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                    image_buf.pop();
                image_msg = image_buf.front();
                image_buf.pop();

                while (point_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                    point_buf.pop();
                point_msg = point_buf.front();
                point_buf.pop();
            }
        }
        m_buf.unlock();

        if (pose_msg != NULL)
        {

            if (skip_first_cnt < SKIP_FIRST_CNT)
            {
                skip_first_cnt++;
                continue;
            }

            if (skip_cnt < SKIP_CNT) 
            {
                skip_cnt++;
                continue;
            }
            else
            {
                skip_cnt = 0;
            }

            cv_bridge::CvImageConstPtr ptr;
            if (image_msg->encoding == "8UC1")
            {
                sensor_msgs::Image img;
                img.header = image_msg->header;
                img.height = image_msg->height;
                img.width = image_msg->width;
                img.is_bigendian = image_msg->is_bigendian;
                img.step = image_msg->step;
                img.data = image_msg->data;
                img.encoding = "mono8";
                ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            }
            else if(image_msg->encoding == "mono8"){
                ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
            }
            else
                ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
            
            cv::Mat image = ptr->image;

            if(!( image.rows==ROW &&  image.cols==COL)){//solve image size problem
                resize(image,image, cv::Size(COL, ROW));
            }
           

            Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                                  pose_msg->pose.pose.position.y,
                                  pose_msg->pose.pose.position.z);
            Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                                     pose_msg->pose.pose.orientation.x,
                                     pose_msg->pose.pose.orientation.y,
                                     pose_msg->pose.pose.orientation.z).toRotationMatrix();
            if((T - last_t).norm() > SKIP_DIS) 
            {
                vector<cv::Point3f> point_3d; 
                vector<cv::Point2f> point_2d_uv; 
                vector<cv::Point2f> point_2d_normal;
                vector<double> point_id;

                for (unsigned int i = 0; i < point_msg->points.size(); i++)
                {
                    cv::Point3f p_3d;
                    p_3d.x = point_msg->points[i].x;
                    p_3d.y = point_msg->points[i].y;
                    p_3d.z = point_msg->points[i].z;
                    point_3d.push_back(p_3d);

                    cv::Point2f p_2d_uv, p_2d_normal;
                    double p_id;
                    p_2d_normal.x = point_msg->channels[i].values[0];
                    p_2d_normal.y = point_msg->channels[i].values[1];
                    p_2d_uv.x = point_msg->channels[i].values[2];
                    p_2d_uv.y = point_msg->channels[i].values[3];
                    p_id = point_msg->channels[i].values[4];
                    point_2d_normal.push_back(p_2d_normal);
                    point_2d_uv.push_back(p_2d_uv);
                    point_id.push_back(p_id);

                }

                KeyFrame* keyframe = new KeyFrame(pose_msg->header.stamp.toSec(), frame_index, T, R, image,
                                   point_3d, point_2d_uv, point_2d_normal, point_id, sequence);   // construct KF
              
                m_process.lock();
                start_flag = 1;
                posegraph.addKeyFrame(keyframe, 1); 
                m_process.unlock();
                frame_index++;
                last_t = T;
            }
        }

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}


void command()
{
    ROS_INFO("press the s button to save the pose graph");
    if (!LOOP_CLOSURE)
        return;
    while(1)
    {
        char c = getchar();
        if (c == 's') 
        {
            m_process.lock();
            posegraph.savePoseGraph();
            m_process.unlock();
            printf("save pose graph finish\nyou can set 'load_previous_pose_graph' to 1 in the config file to reuse it next time\n");

        }

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_graph");
    ros::NodeHandle n("~");
    posegraph.registerPub(n);

    n.getParam("visualization_shift_x", VISUALIZATION_SHIFT_X);
    n.getParam("visualization_shift_y", VISUALIZATION_SHIFT_Y);
    n.getParam("skip_cnt", SKIP_CNT);
    n.getParam("skip_dis", SKIP_DIS); 
    std::string config_file;
    n.getParam("config_file", config_file);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    double camera_visual_size = fsSettings["visualize_camera_size"];
    cameraposevisual.setScale(camera_visual_size);
    cameraposevisual.setLineWidth(camera_visual_size / 10.0);

    TS_LK_THRESHOLD= fsSettings["TS_LK_threshold"];

    LOOP_CLOSURE = fsSettings["loop_closure"];
    std::string LOOP_TOPIC;// loop topic
    int LOAD_PREVIOUS_POSE_GRAPH;
    if (LOOP_CLOSURE)
    {
        // image resolution
        ROW = fsSettings["image_height"];
        COL = fsSettings["image_width"];

        // DBoW2
        std::string pkg_path = ros::package::getPath("pose_graph");
        string vocabulary_file = pkg_path + "/support_files/brief_k10L6.bin";
        cout << "vocabulary_file" << vocabulary_file << endl;
        posegraph.loadVocabulary(vocabulary_file); // load vocabulary

        // retrieve BRIEF description
        BRIEF_PATTERN_FILE = pkg_path + "/support_files/brief_pattern.yml"; // brief pattern file
        cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << endl;

        // create camera model
        system_mode = fsSettings["system_mode"];// 0 for ESIO, 1 for ESVIO

        int pn = config_file.find_last_of('/');
        std::string configPath = config_file.substr(0, pn);
        std::string cam0Calib;
        if (system_mode == 1){
            fsSettings["cam_left_calib"] >> cam0Calib;
        }
        else if (system_mode == 0){
            fsSettings["event_left_calib"] >> cam0Calib;
        }
        std::string cam0Path = configPath + "/" + cam0Calib;
        printf("cam calib path: %s\n", cam0Path.c_str());
        m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(cam0Path.c_str());// camera model

        fsSettings["loop_closure_topic"] >> LOOP_TOPIC;
        fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
        fsSettings["output_path"] >> ESVIO_RESULT_PATH;
        fsSettings["save_image"] >> DEBUG_IMAGE;
        fsSettings["save_loop_match"]>>SAVE_LOOP_MATCH;

        // create folder if not exists
        FileSystemHelper::createDirectoryIfNotExists(POSE_GRAPH_SAVE_PATH.c_str());
        FileSystemHelper::createDirectoryIfNotExists(ESVIO_RESULT_PATH.c_str());

        VISUALIZE_IMU_FORWARD = fsSettings["visualize_imu_forward"];// whether visualize imu forward pose
        LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"]; // whether load previous pose graph
        FAST_RELOCALIZATION = fsSettings["fast_relocalization"];// whether use fast relocalization

        ESVIO_RESULT_PATH = ESVIO_RESULT_PATH + "/esvio_result_loop.txt";
        std::ofstream fout(ESVIO_RESULT_PATH, std::ios::out);
        fout.close();

        std::ofstream fout1(ESVIO_RESULT_PATH, std::ios::app);
            fout.setf(std::ios::fixed, std::ios::floatfield);
            fout << "timestamp" << ",";
            fout << "tx" << ","
                    << "ty" << ","
                    << "tz" << ","
                    << "qx" << ","
                    << "qy" << ","
                    << "qz" << "," 
                    << "qw" << ","<< std::endl;
        fout1.close();
        fsSettings.release();

        if (LOAD_PREVIOUS_POSE_GRAPH)
        {
            printf("load pose graph\n");
            m_process.lock();
            posegraph.loadPoseGraph();
            m_process.unlock();
            printf("load pose graph finish\n");
            load_flag = 1;
        }
        else
        {
            printf("no previous pose graph\n");
            load_flag = 1;
        }
    }

    fsSettings.release();


    // Stereo pose graph
    ros::Subscriber sub_imu_forward = n.subscribe("/stereo_esvio_estimator/imu_propagate", 2000, imu_forward_callback);//pub imu-rate odom
    ros::Subscriber sub_vio = n.subscribe("/stereo_esvio_estimator/odometry", 2000, vio_callback);
    ros::Subscriber sub_image = n.subscribe(LOOP_TOPIC, 2000, loop_callback);
    ros::Subscriber sub_pose = n.subscribe("/stereo_esvio_estimator/keyframe_pose", 2000, pose_callback);
    ros::Subscriber sub_extrinsic = n.subscribe("/stereo_esvio_estimator/extrinsic", 2000, extrinsic_callback);
    ros::Subscriber sub_point = n.subscribe("/stereo_esvio_estimator/keyframe_point", 2000, point_callback);
    ros::Subscriber sub_relo_relative_pose = n.subscribe("/stereo_esvio_estimator/relo_relative_pose", 2000, relo_relative_pose_callback);
    ros::Subscriber sub_event_feature=n.subscribe("/stereo_event_tracker/event_feature_loop", 2000, &eventsfeatureCallback);//callback event-corner feature
    
    // pub topic
    pub_latest_loop_evio = n.advertise<geometry_msgs::PoseStamped>("imu_evio_loop", 1000);//pub latest imu-loop odom
    pub_flight_evio=n.advertise<nav_msgs::Odometry>("evio_odometry", 1000);//pub odom for flight
    pub_match_img = n.advertise<sensor_msgs::Image>("match_image", 1000);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
    pub_key_odometrys = n.advertise<visualization_msgs::Marker>("key_odometrys", 1000);
    pub_vio_path = n.advertise<nav_msgs::Path>("no_loop_path", 1000);
    pub_match_points = n.advertise<sensor_msgs::PointCloud>("match_points", 100);//pub loop

    std::thread measurement_process= std::thread(process);//loop main function
    // std::thread keyboard_command_process = std::thread(command);// keyboard command (push "s" to save pose graph)

    ros::spin();
    return 0;
}

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include "feature_tracker.h"

#include "dvs_msgs/Event.h"
#include "dvs_msgs/EventArray.h"
#include "utility/visualization.h"
#include "tic_toc.h"

#include <queue>
#include <thread>
#include <mutex>
#include <tuple> 

#include <condition_variable>
#include <cmath>
#include "../../esvio_estimator/src/utility/utility.h" 

#define SHOW_UNDISTORTION 0

std::condition_variable con;

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::ImageConstPtr> img_left_buf;
queue<sensor_msgs::ImageConstPtr> img_right_buf;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<nav_msgs::Odometry::ConstPtr> odom_buffer_;//save the state from the back end
std::mutex m_buf;//lock the mutex, push the message to the buffer, unlock the mutex.
std::mutex m_buf_feature;
std::mutex m_buf_event;
std::mutex m_buf_odo;

ros::Publisher pub_event_match;
ros::Publisher pub_event;
ros::Publisher pub_event_restart;

FeatureTracker trackerData;//define the tracker for each camera
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;
double last_imu_t = 0;

Eigen::Vector3f v_cur;
Eigen::Vector3f v_pre;
double t_pre, t_cur;

void sync_process();

queue<dvs_msgs::EventArray> events_left_buf;
queue<dvs_msgs::EventArray> events_right_buf;

bool is_nolinear=false;//whether in the nonlinear optimization state

void pubTrackImage(const cv::Mat &imgTrack, const cv::Mat &imgTrack_two, const cv::Mat &imgTrack_two_point, const cv::Mat &time_surface_map, const double t, const cv::Mat &event_loop) //const dvs_msgs::EventArray &event_left
{//pub the track image
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(t);

    if(!imgTrack.empty())
    {
        sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
        pub_match.publish(imgTrackMsg);//visualize the tracking result of single frame
    }

    if(!imgTrack_two.empty())
    {
        sensor_msgs::ImagePtr imgTrackMsg_two = cv_bridge::CvImage(header, "bgr8", imgTrack_two).toImageMsg();
        pub_match_two.publish(imgTrackMsg_two);//visualize the tracking result of two frames
    }

    if(!imgTrack_two_point.empty())
    {
        sensor_msgs::ImagePtr imgTrackMsg_two_point = cv_bridge::CvImage(header, "bgr8", imgTrack_two_point).toImageMsg();
        pub_match_two_point.publish(imgTrackMsg_two_point);//visulize the matching result of two frames
    }

    if(!time_surface_map.empty())
    {
        sensor_msgs::ImagePtr TimeSurfaceImg = cv_bridge::CvImage(header, "mono8", time_surface_map).toImageMsg();
        pub_time_surface.publish(TimeSurfaceImg);//visulize the time surface
    }

    if(!event_loop.empty())
    {
        sensor_msgs::ImagePtr event_loop_Msg = cv_bridge::CvImage(header, "bgr8", event_loop).toImageMsg();
        pub_event_loop.publish(event_loop_Msg);//visulize the tracking result of single frame
    }
    
}

void state_callback(const nav_msgs::Odometry::ConstPtr &odometry){
    m_buf_odo.lock();
    odom_buffer_.push(odometry);
    m_buf_odo.unlock();
    is_nolinear=true;
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();

    m_buf.lock();
    imu_buf.push(imu_msg);//push imu to the buffer
    m_buf.unlock();
    con.notify_one();

    last_imu_t = imu_msg->header.stamp.toSec();

}


void event_callback_left(const dvs_msgs::EventArray &event_msg){
    m_buf_event.lock();
    if (!events_left_buf.empty())
      events_left_buf.pop();
    events_left_buf.push(event_msg);
    m_buf_event.unlock();
}

void event_callback_right(const dvs_msgs::EventArray &event_msg){
    m_buf_event.lock();
    if (!events_right_buf.empty())
      events_right_buf.pop();
    events_right_buf.push(event_msg);
    m_buf_event.unlock();
}


void handle_stereo_event(const dvs_msgs::EventArray &event_left, const dvs_msgs::EventArray &event_right, double msg_timestamp)
{   
    static int cnt = 0;
    const int n_event = event_left.events.size();

    if(n_event == 0){
        ROS_WARN("not event, please move the event camera or check whether connecting");
        return;
    }

    if(first_image_flag)// the operation for the first frame
    {
        first_image_flag = false;
        first_image_time = msg_timestamp;
        last_image_time = msg_timestamp;
        return;
    }
    // detect unstable camera stream, check the timestamp
    if (msg_timestamp - last_image_time > 1.0 || msg_timestamp < last_image_time)
    {
        ROS_WARN("event stream discontinue! reset the event feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);// reset para and reset operation
        return;
    }
    last_image_time = msg_timestamp;

    // frequency control
    if (round(1.0 * pub_count / (msg_timestamp - first_image_time)) <= FREQ) 
    {
        PUB_THIS_FRAME = true;// pub the current frame
        // reset the frequency control
        if (abs(1.0 * pub_count / (msg_timestamp - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = msg_timestamp;
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    double msg_timestamp_left = event_left.events.back().ts.toSec();

    if (Do_motion_correction == 0){
        trackerData.trackEvent(msg_timestamp_left, event_left, event_right);//optiflow track
    }else{
        Motion_correction_value motion_compensation;
        double t_left_0 = event_left.events[0].ts.toSec();//the time of the first event in the left event camera
        double t_left_1 = event_left.header.stamp.toSec();//the time of the current event in the left event camera
        Eigen::Vector3d temp_v;//current velocity
        Eigen::Vector3f temp_a;//current acceleration
        double temp_time;//current time
        Eigen::Vector4d State_;//save the current state and time
        Eigen::Vector2d t_0_1(t_left_0,t_left_1);//push t_left_0 and t_left_1 together
        Eigen::Vector3f omega_avg_;//imu angular velocity
        Eigen::Vector3f accel_avg_;//imu acceleration

        if (!imu_buf.empty()){

            if(!odom_buffer_.empty()){
                temp_time = odom_buffer_.front()->header.stamp.toSec();
                temp_v[0]=odom_buffer_.front()->twist.twist.linear.x;//current velocity of the drone
                temp_v[1]=odom_buffer_.front()->twist.twist.linear.y;
                temp_v[2]=odom_buffer_.front()->twist.twist.linear.z;
                odom_buffer_.pop();//pop the front element

                State_[0]=temp_v[0];
                State_[1]=temp_v[1];
                State_[2]=temp_v[2];
                State_[3]=temp_time;

                v_pre[0] = v_cur[0];
                v_pre[1] = v_cur[1];
                v_pre[2] = v_cur[2];
                v_cur[0] = temp_v[0];
                v_cur[1] = temp_v[1];
                v_cur[2] = temp_v[2];

                t_pre = t_cur;
                t_cur = temp_time;

                temp_a[0] = (v_cur[0] - v_pre[0])/(t_cur - t_pre);
                temp_a[1] = (v_cur[1] - v_pre[1])/(t_cur - t_pre);
                temp_a[2] = (v_cur[2] - v_pre[2])/(t_cur - t_pre);

            }

            while(!imu_buf.empty()){
                if (imu_buf.front()->header.stamp.toSec() < t_left_0)//remove the imu data before the first event
                    imu_buf.pop();
                else
                    break;
            }

            if (!imu_buf.empty()){
                omega_avg_[0]=imu_buf.front()->angular_velocity.x;
                omega_avg_[1]=imu_buf.front()->angular_velocity.y;
                omega_avg_[2]=imu_buf.front()->angular_velocity.z;
                accel_avg_[0]=imu_buf.front()->linear_acceleration.x;
                accel_avg_[1]=imu_buf.front()->linear_acceleration.y;
                accel_avg_[2]=imu_buf.front()->linear_acceleration.z-9.805;
            }
        }
        motion_compensation = std::make_pair(is_nolinear,std::make_pair(std::make_pair (State_,v_pre),std::make_pair(t_0_1,std::make_pair(temp_a,omega_avg_))));

        trackerData.trackEvent(msg_timestamp_left, event_left, event_right, motion_compensation); //optiflow track
    }

    if (SHOW_TRACK)//show the tracking process
        {
            cv::Mat imageTrack=trackerData.getTrackImage();
            cv::Mat imgTrack_two =trackerData.getTrackImage_two();
            cv::Mat imgTrack_two_point =trackerData.getTrackImage_two_point();
            cv::Mat Time_surface_map =trackerData.gettimesurface();
            cv::Mat event_loop = trackerData.getEventloop();
            pubTrackImage(imageTrack,imgTrack_two,imgTrack_two_point,Time_surface_map,last_image_time,event_loop);
        }


   if (PUB_THIS_FRAME)
   {
        pub_count++;//for frequency control

        {
            sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
            sensor_msgs::ChannelFloat32 id_of_point;
            sensor_msgs::ChannelFloat32 u_of_point;
            sensor_msgs::ChannelFloat32 v_of_point;
            sensor_msgs::ChannelFloat32 velocity_x_of_point;
            sensor_msgs::ChannelFloat32 velocity_y_of_point;

            feature_points->header.stamp = ros::Time(msg_timestamp);
            feature_points->header.frame_id = "world";

            int camera_id = 0;
            int feature_id;
            geometry_msgs::Point32 p;
            
            //features on the left image
            set<int> hash_ids;
            for(size_t j=0; j<trackerData.ids.size(); j++){
               if (trackerData.track_cnt[j] > 1)//
                    {
                        feature_id = trackerData.ids[j];
                        p.x = trackerData.cur_un_pts[j].x;
                        p.y = trackerData.cur_un_pts[j].y;
                        p.z = 1;
                        
                        hash_ids.insert(feature_id);
                        feature_points->points.push_back(p);
                        id_of_point.values.push_back(feature_id * NUM_OF_CAM_stereo + camera_id);
                        u_of_point.values.push_back(trackerData.cur_pts[j].x);
                        v_of_point.values.push_back(trackerData.cur_pts[j].y);
                        velocity_x_of_point.values.push_back(trackerData.pts_velocity[j].x);
                        velocity_y_of_point.values.push_back(trackerData.pts_velocity[j].y);
                    } 
            }
            
            //features on the right image
            camera_id = 1;
            for(size_t j=0; j<trackerData.ids_right.size(); j++){
               feature_id = trackerData.ids_right[j];
               if (hash_ids.find(feature_id) != hash_ids.end())
                    {
                        p.x = trackerData.cur_un_right_pts[j].x;
                        p.y = trackerData.cur_un_right_pts[j].y;
                        p.z = 1;//normalized plane point
                        feature_points->points.push_back(p);
                        id_of_point.values.push_back(feature_id * NUM_OF_CAM_stereo + camera_id);
                        u_of_point.values.push_back(trackerData.cur_right_pts[j].x);
                        v_of_point.values.push_back(trackerData.cur_right_pts[j].y);
                        velocity_x_of_point.values.push_back(trackerData.right_pts_velocity[j].x);
                        velocity_y_of_point.values.push_back(trackerData.right_pts_velocity[j].y);
                    } 
            }
            
            feature_points->channels.push_back(id_of_point);
            feature_points->channels.push_back(u_of_point);
            feature_points->channels.push_back(v_of_point);
            feature_points->channels.push_back(velocity_x_of_point);
            feature_points->channels.push_back(velocity_y_of_point);
            // ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());
            // ROS_INFO("event cnt = %d publish %lf with %d features", ++cnt, feature_points->header.stamp.toSec(), feature_points->points.size());

            // skip the first image; since no optical speed on frist image
            if (!init_pub)
            {
                init_pub = 1;
            }
            else
                pub_img.publish(feature_points);//pub features to pose graph optimization
        }
         
    }

}


int main(int argc, char **argv)
{
    ROS_WARN("into event point feature detection and tracking");
    ros::init(argc, argv, "stereo_event_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    
    readParameters_event(n);

    trackerData.stereo_readIntrinsicParameter(CAM_NAMES);  // get the intrinsic parameter of the camera

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());// imu callback
    ros::Subscriber vio_sub=n.subscribe("/stereo_esvio_estimator/odometry",1000,state_callback);

    ros::Subscriber sub_event_left = n.subscribe(EVENT_LEFT, 20000, &event_callback_left);
    ros::Subscriber sub_event_right = n.subscribe(EVENT_RIGHT, 20000, &event_callback_right);

    registerPub(n);//register the publisher

    std::thread sync_thread{sync_process};

    ros::spin(); 
    return 0;
}

void sync_process()
{
    while(1)
    {
            dvs_msgs::EventArray event_left, event_right;
            std_msgs::Header header;
            double msg_timestamp = 0.0;

            m_buf_event.lock();
            if (!events_left_buf.empty() && !events_right_buf.empty()){
                double time_left = events_left_buf.front().header.stamp.toSec();
                double time_right = events_right_buf.front().header.stamp.toSec();
                msg_timestamp = time_left;
                if(time_left < time_right - 0.2) //tolerance
                {
                    events_left_buf.pop();
                    printf("throw events1\n");
                }
                else if(time_left > time_right + 0.2)
                {
                    events_right_buf.pop();
                    printf("throw events2\n");
                }
                else
                {
                    msg_timestamp = events_left_buf.front().header.stamp.toSec();
                    header = events_left_buf.front().header;
                    event_left = events_left_buf.front();
                    events_left_buf.pop();
                    event_right = events_right_buf.front();
                    events_right_buf.pop();
                }

            }
            m_buf_event.unlock();

            if(event_left.events.size()!= 0){

                handle_stereo_event(event_left, event_right, msg_timestamp);

            }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
    

}
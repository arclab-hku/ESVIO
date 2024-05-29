#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>

#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::ImageConstPtr> img_left_buf;
queue<sensor_msgs::ImageConstPtr> img_right_buf;
std::mutex m_buf;

ros::Publisher pub_img,pub_match;
ros::Publisher pub_restart;
ros::Publisher pub_event_loop;

FeatureTracker trackerData; // [NUM_OF_CAM_stereo];
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;

void sync_process(); 
cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg); 

void img_callback_left(const sensor_msgs::ImageConstPtr &img_msg){
    m_buf.lock();
    if (!img_left_buf.empty())
      img_left_buf.pop();
    img_left_buf.push(img_msg);
    m_buf.unlock();
}
void img_callback_right(const sensor_msgs::ImageConstPtr &img_msg){
    m_buf.lock();
    if (!img_right_buf.empty())
      img_right_buf.pop();
    img_right_buf.push(img_msg);
    m_buf.unlock(); 
}


void handle_stereo_image(cv::Mat& img_left, cv::Mat& img_right, double msg_timestamp)
{
    static int cnt = 0;
    // ROS_WARN("received img_msg timestamp: %lf", msg_timestamp);
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = msg_timestamp; // img_msg->header.stamp.toSec();
        last_image_time = msg_timestamp; // img_msg->header.stamp.toSec();
        return;
    }
    // detect unstable camera stream
    if(msg_timestamp - last_image_time > 1.0 || msg_timestamp < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }
    last_image_time = msg_timestamp; 

    // frequency control
    if (round(1.0 * pub_count / (msg_timestamp - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (msg_timestamp - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = msg_timestamp; 
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;
        
    if(EQUALIZE){
         static cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
         clahe->apply(img_left, img_left); 
         clahe->apply(img_right, img_right); 
    }

    TicToc t_r;   

    trackerData.trackImage(msg_timestamp, img_left, img_right); //problem here

    if(SHOW_TRACK){
        cv::Mat imgTrack = trackerData.getTrackImage(); 

        std_msgs::Header header;
        header.frame_id = "world";
        header.stamp = ros::Time(msg_timestamp);
        sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
        pub_match.publish(imgTrackMsg);
    }

   if (PUB_THIS_FRAME)
   {
        pub_count++;
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

        // features on the left image 
        set<int> hash_ids; 
        for(size_t j=0; j < trackerData.ids.size(); j++){
            if(trackerData.track_cnt[j] > 1){
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

        // features on the right image 
        camera_id = 1; 
        for(size_t j=0; j < trackerData.ids_right.size(); j++){
            feature_id = trackerData.ids_right[j];
            if(hash_ids.find(feature_id) != hash_ids.end()){
                p.x = trackerData.cur_un_right_pts[j].x;
                p.y = trackerData.cur_un_right_pts[j].y;
                p.z = 1;
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

        // ROS_INFO("cnt = %d publish %lf with %d features", ++cnt, feature_points->header.stamp.toSec(), feature_points->points.size());

        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_img.publish(feature_points);
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_image_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info); // Debug Info
    readParameters(n);

    ROS_WARN("into stereo image feature detection and tracking");

    // setup cameras 
    trackerData.stereo_readIntrinsicParameter(CAM_NAMES); 

    ros::Subscriber sub_img_left = n.subscribe(IMAGE_LEFT, 100, img_callback_left);
    ros::Subscriber sub_img_right = n.subscribe(IMAGE_RIGHT, 100, img_callback_right);

    pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
    pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);
    pub_restart = n.advertise<std_msgs::Bool>("restart",1000);

    std::thread sync_thread{sync_process};
    ros::spin();
    return 0;
}


// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {   
            cv::Mat image_left, image_right;
            std_msgs::Header header;
            double msg_timestamp = 0.0;

            m_buf.lock();
            if (!img_left_buf.empty() && !img_right_buf.empty())
            {
                double time_left = img_left_buf.front()->header.stamp.toSec();
                double time_right = img_right_buf.front()->header.stamp.toSec();
                msg_timestamp = time_left;

                if(time_left <= time_right - 1)// tolerance
                {
                    img_left_buf.pop();
                    printf("throw img_left\n");
                }
                else if(time_left > time_right + 1)
                {
                    img_right_buf.pop();
                    printf("throw img_right\n");
                }
                else
                {
                    msg_timestamp = img_left_buf.front()->header.stamp.toSec();
                    header = img_left_buf.front()->header;
                    image_left = getImageFromMsg(img_left_buf.front());
                    img_left_buf.pop();
                    image_right = getImageFromMsg(img_right_buf.front());
                    img_right_buf.pop();
                    //printf("find img_left and img_right\n");
                }
            }
            m_buf.unlock();
            
            if(!image_left.empty())
                handle_stereo_image(image_left, image_right, msg_timestamp); 

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)//image type and size check
{
    cv_bridge::CvImageConstPtr ptr;
    cv::Mat ret_img;
    if (img_msg->encoding == "8UC1")
    {
        ROS_DEBUG("feature_tracker_node.cpp: image type: gray 8UC1 ");
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        // ret_img = ptr->image.clone();
        ret_img = ptr->image;
    }
    else if(img_msg->encoding == "8UC3"){

        ROS_DEBUG("feature_tracker_node.cpp: image type: RGB 8UC3"); 
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "bgr8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        ret_img = ptr->image.clone(); 
        cv::cvtColor(ret_img, ret_img, cv::COLOR_BGR2GRAY);
    }
    else if(img_msg->encoding == "mono8"){

        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
        ret_img = ptr->image;
    }
    else if(img_msg->encoding == "rgb8"){
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
        ret_img = ptr->image;
        cv::cvtColor(ret_img, ret_img, cv::COLOR_RGB2GRAY);

    }
    else if(img_msg->encoding == "bgr8"){

        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        ret_img = ptr->image;
        cv::cvtColor(ret_img, ret_img, cv::COLOR_BGR2GRAY);
    }
    else{
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
        ret_img = ptr->image;

    }

    if(!(ret_img.rows==ROW && ret_img.cols==COL)){//size check
         resize(ret_img,ret_img, cv::Size(COL, ROW));
    }
    
    return ret_img;
}

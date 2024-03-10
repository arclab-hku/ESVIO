#include "visualization.h"
#include "../dvs_msgs/Event.h"
#include "../dvs_msgs/EventArray.h"

using namespace ros;
using namespace Eigen;
ros::Publisher pub_loop_image;
ros::Publisher pub_img,pub_match, pub_match_two;
ros::Publisher pub_time_surface;
ros::Publisher pub_restart;
ros::Publisher corner_pub;
ros::Publisher pub_match_two_point;
ros::Publisher pub_event_loop;

void registerPub(ros::NodeHandle &n)
{
    pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);//pub /stereo_event_tracker/feature
    pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);
    pub_match_two=n.advertise<sensor_msgs::Image>("feature_img_two",1000);
    pub_time_surface = n.advertise<sensor_msgs::Image>("timesurface_map", 1000);
    pub_restart = n.advertise<std_msgs::Bool>("restart",1000);

    corner_pub= n.advertise<dvs_msgs::EventArray>("event_feature_loop",1000);

    pub_loop_image = n.advertise<sensor_msgs::Image>("loop_image", 1000);

    pub_match_two_point=n.advertise<sensor_msgs::Image>("pointfeature_img_two",1000);
    pub_event_loop = n.advertise<sensor_msgs::Image>("event_loop", 1000);

}

void pubLoopImage(const cv::Mat &timesurface, const double t)
{

    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(t);

    sensor_msgs::ImagePtr TimeSurfaceImg = cv_bridge::CvImage(header, "mono8", timesurface).toImageMsg();
    pub_loop_image.publish(TimeSurfaceImg);
}

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"


Estimator estimator;

std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> eventfeature_buf;
queue<sensor_msgs::PointCloudConstPtr> imagefeature_buf;
queue<sensor_msgs::PointCloudConstPtr> relo_buf;
int sum_of_wait = 0;

std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_estimator;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_image = 0;
bool init_event = 0;
bool init_imu = 1;
double last_imu_t = 0;

void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    // get acc, normal version
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
  
    // flight version
    // double dx = imu_msg->linear_acceleration.z;
    // double dy = -imu_msg->linear_acceleration.x;
    // double dz = -imu_msg->linear_acceleration.y;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    // get gyr, normal version
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;

    // flight version

    // double rx = imu_msg->angular_velocity.z;
    // double ry = -imu_msg->angular_velocity.x;
    // double rz = -imu_msg->angular_velocity.y;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g; 

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;    
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}


void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());

}


std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>,
                std::pair<sensor_msgs::PointCloudConstPtr, sensor_msgs::PointCloudConstPtr> > > 
getMeasurements_event_image_imu()
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>,
                std::pair<sensor_msgs::PointCloudConstPtr, sensor_msgs::PointCloudConstPtr> > > measurements;

    while (true)
    {
        if (imu_buf.empty() || imagefeature_buf.empty())
            return measurements;

        if (!(imu_buf.back()->header.stamp.toSec() > imagefeature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            sum_of_wait++;
            return measurements;
        }

        if (!(imu_buf.front()->header.stamp.toSec() < imagefeature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("throw event and img, only should happen at the beginning");
            if(eventfeature_buf.size()!=0)
                eventfeature_buf.pop();
            if(imagefeature_buf.size()!=0)
                imagefeature_buf.pop();
            continue;
        } 

        sensor_msgs::PointCloudConstPtr event_msg;
        if(eventfeature_buf.size()!=0){
            event_msg = eventfeature_buf.front();
            eventfeature_buf.pop();
        }

        sensor_msgs::PointCloudConstPtr image_msg;
        if(imagefeature_buf.size()!=0){
            image_msg = imagefeature_buf.front();
            imagefeature_buf.pop();
        }
       
        
        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp.toSec() < image_msg->header.stamp.toSec() + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }

        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            ROS_WARN("no imu between two events");

        measurements.emplace_back(IMUs, std::make_pair(image_msg, event_msg));
    }
    return measurements;
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
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    last_imu_t = imu_msg->header.stamp.toSec();

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR){
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);//IMU-rate update
        }
    }


}


void stereo_eventfeature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    if (!init_event)
    {
        init_event = 1;
        return;
    }
        m_buf.lock();
        eventfeature_buf.push(feature_msg);
        m_buf.unlock();
        con.notify_one();
    // ROS_INFO("no enough event feature, please move the camera");
}


void stereo_imagefeature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    if (!init_image)
    {
        init_image = 1;
        return;
    }
    m_buf.lock();
    imagefeature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the EVIO estimator!");
        m_buf.lock();
        while(!eventfeature_buf.empty())
            eventfeature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        while(!imagefeature_buf.empty())
            imagefeature_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}

void relocalization_callback(const sensor_msgs::PointCloudConstPtr &points_msg)
{
    printf("relocalization callback! \n");
    m_buf.lock();
    relo_buf.push(points_msg);
    m_buf.unlock();
}


void process_event_imu()
{
    while (true)
    {
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>,
                std::pair<sensor_msgs::PointCloudConstPtr, sensor_msgs::PointCloudConstPtr> > > measurements; 

        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements_event_image_imu()).size() != 0;
                 });
        lk.unlock();

        m_estimator.lock();
        for (auto &measurement : measurements) 
        {
            auto img_msg = measurement.second.first;//image feature point
            auto event_msg = measurement.second.second;//event feature point
      
            // ROS_INFO("The number of the event point:%d",event_msg->points.size());
            // ROS_INFO("The number of the image point:%d",img_msg->points.size());

            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first)
            {
                double t = imu_msg->header.stamp.toSec();
                double img_t = img_msg->header.stamp.toSec() + estimator.td;

                // ROS_INFO("processing imu data with stamp %f \n", imu_msg->header.stamp.toSec());
                // ROS_INFO("processing img data with stamp %f \n", img_msg->header.stamp.toSec());

                if (t <= img_t)
                { 
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    ROS_ASSERT(dt >= 0);
                    current_time = t;
                    // normal version
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;

                    // flight version
                    // dx = imu_msg->linear_acceleration.z;
                    // dy = -imu_msg->linear_acceleration.x;
                    // dz = -imu_msg->linear_acceleration.y;
                    // rx = imu_msg->angular_velocity.z;
                    // ry = -imu_msg->angular_velocity.x;
                    // rz = -imu_msg->angular_velocity.y;

                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);

                }
                else 
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    // normal version
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;

                    // flight version
                    // dx = w1 * dx + w2 * imu_msg->linear_acceleration.z;
                    // dy = w1 * dy + w2 * -imu_msg->linear_acceleration.x;
                    // dz = w1 * dz + w2 * -imu_msg->linear_acceleration.y;
                    // rx = w1 * rx + w2 * imu_msg->angular_velocity.z;
                    // ry = w1 * ry + w2 * -imu_msg->angular_velocity.x;
                    // rz = w1 * rz + w2 * -imu_msg->angular_velocity.y;
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }

            sensor_msgs::PointCloudConstPtr relo_msg = NULL;
            while (!relo_buf.empty())  
            {
                relo_msg = relo_buf.front();
                relo_buf.pop();
            }
            if (relo_msg != NULL)  
            {
                vector<Vector3d> match_points; 
                double frame_stamp = relo_msg->header.stamp.toSec();
                for (unsigned int i = 0; i < relo_msg->points.size(); i++)
                {
                    Vector3d u_v_id;
                    u_v_id.x() = relo_msg->points[i].x;
                    u_v_id.y() = relo_msg->points[i].y;
                    u_v_id.z() = relo_msg->points[i].z;
                    match_points.push_back(u_v_id);
                }

                Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg->channels[0].values[7];
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
            }

            // ROS_INFO("processing img data with stamp %f \n", img_msg->header.stamp.toSec());
            // ROS_INFO("processing event data with stamp %f \n", event_msg->header.stamp.toSec());

            TicToc t_s;
            
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> event;
            if(event_msg!=NULL){
                for (unsigned int i = 0; i < event_msg->points.size(); i++)
                {
                    int v = event_msg->channels[0].values[i] + 0.5;
                    int feature_id = v / NUM_OF_CAM_stereo;
                    int camera_id = v % NUM_OF_CAM_stereo;
                    double x = event_msg->points[i].x;
                    double y = event_msg->points[i].y;
                    double z = event_msg->points[i].z;
                    double p_u = event_msg->channels[1].values[i];
                    double p_v = event_msg->channels[2].values[i];
                    double velocity_x = event_msg->channels[3].values[i];
                    double velocity_y = event_msg->channels[4].values[i];
                    ROS_ASSERT(z == 1);
                    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                    xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                    event[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
                }
            }

            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            if(img_msg!=NULL){
                for (unsigned int i = 0; i < img_msg->points.size(); i++)
                {
                    int v = img_msg->channels[0].values[i] + 0.5;
                    int feature_id = v / NUM_OF_CAM_stereo;
                    int camera_id = v % NUM_OF_CAM_stereo;
                    double x = img_msg->points[i].x;
                    double y = img_msg->points[i].y;
                    double z = img_msg->points[i].z;
                    double p_u = img_msg->channels[1].values[i];
                    double p_v = img_msg->channels[2].values[i];
                    double velocity_x = img_msg->channels[3].values[i];
                    double velocity_y = img_msg->channels[4].values[i];
                    ROS_ASSERT(z == 1);
                    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                    xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                    image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
                }
            }

            // ROS_INFO("The number of the event point:%d",event_msg->points.size());
            // ROS_INFO("The number of the image point:%d",img_msg->points.size());
            
            estimator.Stereo_processVisual(image, event, img_msg->header);

            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t);
            std_msgs::Header header = img_msg->header;
            header.frame_id = "world";

            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);

            if (relo_msg != NULL)
                pubRelocalization(estimator);
        }
        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_esio_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);
    estimator.setParameter();
    estimator.readIntrinsicParameter(CAM_NAMES);

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    ROS_WARN("waiting for event and imu...");

    registerPub(n);

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_imagefeature = n.subscribe("/stereo_event_tracker/feature", 2000, stereo_imagefeature_callback);//event node
    ros::Subscriber sub_relo_points = n.subscribe("/pose_graph/match_points", 2000, relocalization_callback);

    std::thread measurement_process{process_event_imu};
    ros::spin();

    return 0;
}

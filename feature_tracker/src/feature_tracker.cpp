#include "feature_tracker.h"
#include "event_detector/event_detector.h"
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>

esvio::EventDetector detector = esvio::EventDetector();//declare the detector

int FeatureTracker::n_id = 0;//no. of left features
int FeatureTracker::n_id_right = 0;//no. of right features
std::mutex mutex_threads;//create a mutex

void Event_FeaturesToTrack( const dvs_msgs::EventArray &last_event,  vector<cv::Point2f> &n_pts, const vector<cv::Point2f> &_cur_pts, const int maxCorners, const int _MIN_DIST, const cv::Mat event_mask, const cv::Mat time_surface_map)
{
    int ncorners = 0;//cal no. of corners
    n_pts.clear();
    cv:: Mat event_mask_cur=event_mask.clone();//value of 255.0 is not detected

    if(maxCorners>0){//when need to detect
    for (const dvs_msgs::Event& e : last_event.events) { // get each event in last_event
        if (ncorners>=maxCorners) {//if greater than the required points, then exit
                break;
            } 
            else{
                if(event_mask_cur.at<double>(e.y,e.x)!=255.0 ){
                     if(time_surface_map.at<uchar>(e.y,e.x) !=TS_LK_THRESHOLD){
                          if (detector.isCorner(e.ts.toSec(), e.x, e.y, e.polarity)){//if corner, then put in n_pts and count
                                n_pts.push_back(cv::Point2f((float)e.x, (float)e.y));
                                ncorners++;//count the number of corners, if greater than a theshold, then exit
                                cv::circle(event_mask_cur, cv::Point(e.x,e.y), _MIN_DIST, 255.0, -1);//draw a solid circle at e.x and e.y and assign 255, so that this range will not be selected next time
                          }
                     }                
                }
            }

        }            
    }              
}

bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

bool inBorder_event(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL_event - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW_event - BORDER_SIZE;
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(std::vector<cv::DMatch> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}


FeatureTracker::FeatureTracker()
{
}

void FeatureTracker::Image_setMask()
{
    if(FISHEYE)
        mask_image = fisheye_mask.clone();
    else
        mask_image = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));

    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < cur_pts.size(); i++){
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));
    }

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    cur_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask_image.at<uchar>(it.second.first) == 255)
        {
            cur_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask_image, it.second.first, MIN_DIST_IMG, 0, -1);
        }
    }
}


void FeatureTracker::Event_setMask()
{
    mask_event = cv::Mat(ROW_event, COL_event, CV_64FC1, cv::Scalar(0.0));

    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < cur_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    cur_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask_event.at<double>(it.second.first) == 0.0)
        {
            cur_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask_event, it.second.first, MIN_DIST, 255.0, -1);
        }
    }
}

void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}

//stereo feature tracking
void FeatureTracker::trackImage(double _cur_time, const cv::Mat &img_left, const cv::Mat &img_right)
{   
    TicToc t_r;
    cur_time = _cur_time;
    int row = img_left.rows;
    int col = img_left.cols;

    cv::Mat rightImg = img_right;

    if(cur_img_left.empty()){
        prev_img_left = cur_img_left = img_left; 
    }else{
        cur_img_left = img_left; 
    }
    cur_pts.clear();
    
    if (prev_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        cv::calcOpticalFlowPyrLK(prev_img_left, cur_img_left, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);

        if(FLOW_BACK)
        {
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts;
            cv::calcOpticalFlowPyrLK(cur_img_left, prev_img_left, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3);// 
            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }
        
        for (int i = 0; i < int(cur_pts.size()); i++)
            if (status[i] && !inBorder(cur_pts[i]))
                status[i] = 0;
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(ids, status );
        reduceVector(track_cnt, status);
    }

    for (auto &n : track_cnt)
        n++;
    
    if (PUB_THIS_FRAME)
    {        
        TicToc t_m;
        Image_setMask();

        TicToc t_t;
        int n_max_cnt = MAX_CNT_IMG - static_cast<int>(cur_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask_image.empty())
                cout << "mask is empty " << endl;
            if (mask_image.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            cv::goodFeaturesToTrack(cur_img_left, n_pts, MAX_CNT_IMG - cur_pts.size(), 0.01, MIN_DIST_IMG, mask_image);

        }
        else
            n_pts.clear();
         
        for (auto &p : n_pts)
        {
            cur_pts.push_back(p);
            ids.push_back(n_id++);
            track_cnt.push_back(1);
        }
    }
    
    cur_un_pts.clear();
    cur_un_pts = undistortedPts(cur_pts, stereo_m_camera[0]);
    pts_velocity.clear();
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

    if(!img_right.empty())
    {
        ids_right.clear();
        cur_right_pts.clear();
        cur_un_right_pts.clear();
        right_pts_velocity.clear();
        cur_un_right_pts_map.clear();
        if(!cur_pts.empty())
        {
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            vector<float> err;
            // cur left ---- cur right
            cv::calcOpticalFlowPyrLK(cur_img_left, rightImg, cur_pts, cur_right_pts, status, err, cv::Size(21, 21), 3);// 3
            
            // reverse check cur right ---- cur left
            if(FLOW_BACK && !cur_right_pts.empty())
            {
                cv::calcOpticalFlowPyrLK(rightImg, cur_img_left, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
                for(size_t i = 0; i < status.size(); i++)
                {
                    if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
                    {
                        status[i] = 1;
                    }
                    else
                        status[i] = 0;
                }
            }

            // //fundamental matrix check
            
                // vector<cv::Point2f> un_l_pts, un_r_pts; 
                // un_l_pts.reserve(ids.size()); 
                // un_r_pts.reserve(ids.size()); 

                // for(int i=0; i<ids.size(); i++){
                //     if(status[i]){
                //         Eigen::Vector3d tmp_p;
                //         stereo_m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
                //         // ouf2 << i<<" l: "<< tmp_p.transpose()<<" "; 
                //         // tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                //         // tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                //         un_l_pts.push_back(cv::Point2f(tmp_p.x(), tmp_p.y())); 
                //         // ouf2<<" proj: "<<tmp_p.x()<<" "<<tmp_p.y()<<" "<<endl; 

                //         stereo_m_camera[1]->liftProjective(Eigen::Vector2d(cur_right_pts[i].x, cur_right_pts[i].y), tmp_p);
                //         // ouf2 << i<<" r: "<< tmp_p.transpose()<<" "; 
                //         // tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
                //         // tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
                //         un_r_pts.push_back(cv::Point2f(tmp_p.x(), tmp_p.y()));
                //         // ouf2<<" proj: "<<tmp_p.x()<<" "<<tmp_p.y()<<" "<<endl; 
                //         // ouf<<i<<" "<<cur_pts[i].x<<" "<<cur_pts[i].y<<" "<<cur_right_pts[i].x<<" "<<cur_right_pts[i].y<<endl;
                //     }
                // }

            // // use Tlr to to verification

            ids_right = ids;
            reduceVector(cur_right_pts, status);
            reduceVector(ids_right, status);
            reduceVector(track_cnt_right, status);

            for (auto &n : track_cnt_right)
                n++;

            for (auto &p : cur_right_pts)
            {
                track_cnt_right.push_back(1);
            }

            cur_un_right_pts = undistortedPts(cur_right_pts, stereo_m_camera[1]);
            right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);

        }
        prev_un_right_pts_map = cur_un_right_pts_map;
    }
    if(SHOW_TRACK)
        drawTrack(cur_img_left, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);

    prev_img_left = cur_img_left;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;

    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];
    
    return;//
}

void FeatureTracker::trackEvent(double _cur_time, const dvs_msgs::EventArray &event_left, const dvs_msgs::EventArray &event_right)
{
    cur_time = _cur_time;
    cv::Mat img_left;
    cv::Mat img_right;
    cv::Mat rightImg; // for drawTrack

    if(FLAG_DETECTOR_NOSTART){
        FLAG_DETECTOR_NOSTART = false;
        detector.init(COL_event,ROW_event);
    }

    detector.cur_event_mat_left = cv::Mat::zeros(cv::Size(COL_event, ROW_event), CV_8UC3);
    detector.cur_event_mat_right = cv::Mat::zeros(cv::Size(COL_event, ROW_event), CV_8UC3);
    
    TicToc t_SAE;
    for (const dvs_msgs::Event& e_left:event_left.events){
        detector.createSAE_left(e_left.ts.toSec(), e_left.x, e_left.y, e_left.polarity);
    }

    for (const dvs_msgs::Event& e_right:event_right.events){//e_left is a pointer to the event_left.events, traverse the information in event_left.events
        detector.createSAE_right(e_right.ts.toSec(), e_right.x, e_right.y, e_right.polarity);//pointer to the time, x, y, polarity in the event
    }

    cv::Mat event_mat_left = detector.cur_event_mat_left;
    cv::Mat event_mat_right = detector.cur_event_mat_right;

    const cv::Mat time_surface_map_left = detector.SAEtoTimeSurface_left(cur_time);
    const cv::Mat time_surface_map_right = detector.SAEtoTimeSurface_right(cur_time);

    time_surface_visualization_left = time_surface_map_left.clone();
    time_surface_visualization_right = time_surface_map_right.clone();
    cv::Mat time_surface_left = time_surface_map_left.clone();
    cv::Mat time_surface_right = time_surface_map_right.clone();
    
    if (EQUALIZE)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->apply(time_surface_left,img_left);
        clahe->apply(time_surface_right,img_right);
        cv::normalize(img_left, img_left, 0, 255, CV_MINMAX);
        cv::normalize(img_right, img_right, 0, 255, CV_MINMAX);
    }
    else
    {
        img_left = time_surface_left;
        img_right = time_surface_right;
    }


    if(cur_img_left.empty()){
        prev_img_left = cur_img_left = img_left; 
        prev_event_mat_left = event_mat_left; 
    }else{
        cur_img_left = img_left; 
    }
    cur_pts.clear();

    if(cur_img_right.empty()){
        prev_img_right = cur_img_right =img_right;
    }else{
        cur_img_right = img_right;
    }
    cur_right_pts.clear();
    
    if (prev_pts.size() > 0)
    {   
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        cv::calcOpticalFlowPyrLK(prev_img_left, cur_img_left, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);

        // reverse check
        if(FLOW_BACK)
        {
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts;
            cv::calcOpticalFlowPyrLK(cur_img_left, prev_img_left, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }
        
        for (int i = 0; i < int(cur_pts.size()); i++)
            if (status[i] && !inBorder_event(cur_pts[i]))
                status[i] = 0;
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(ids, status );
        reduceVector(track_cnt, status);
    }

    for (auto &n : track_cnt)
        n++;

    if (PUB_THIS_FRAME)
    {
        rejectWithF_event();//F RANSAC
        TicToc t_m;
        Event_setMask();

        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
        if (n_max_cnt > 0)
        {   
            if(mask_event.empty())
                cout << "the time surface mask is empty " << endl;
            if (mask_event.type() != CV_64FC1)
                cout << "time surface mask type wrong " << endl;
            if (mask_event.size() != time_surface_left.size())
                cout << "wrong size" << endl;
            
            Event_FeaturesToTrack(event_left, n_pts, cur_pts, MAX_CNT - cur_pts.size(), MIN_DIST, mask_event, time_surface_left);
        }
        else
            n_pts.clear();

        for (auto &p : n_pts)
        {
            cur_pts.push_back(p);
            ids.push_back(n_id++);
            track_cnt.push_back(1);
        }
    }
    cur_un_pts.clear();//
    cur_un_pts = undistortedPts(cur_pts, stereo_m_camera[0]);
    pts_velocity.clear();//
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

    if(!img_right.empty())
    {
        ids_right.clear();
        cur_right_pts.clear();
        cur_un_right_pts.clear();
        right_pts_velocity.clear();
        cur_un_right_pts_map.clear();
        track_cnt_right.clear();

        if(!cur_pts.empty())
        {
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            vector<float> err;
            // cur left ---- cur right matching
            cv::calcOpticalFlowPyrLK(cur_img_left, cur_img_right, cur_pts, cur_right_pts, status, err, cv::Size(21, 21), 3);
      
            // // reverse check cur right ---- cur left matching
            if(FLOW_BACK && !cur_right_pts.empty())
            {
                cv::calcOpticalFlowPyrLK(cur_img_right, cur_img_left, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
                for(size_t i = 0; i < status.size(); i++)
                {
                    if(status[i] && statusRightLeft[i] && inBorder_event(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
                    {
                        status[i] = 1;
                    }
                    else
                        status[i] = 0;
                }
            }

            ids_right = ids;
            reduceVector(cur_right_pts, status);
            reduceVector(ids_right, status);
            reduceVector(track_cnt_right, status);

            // // fundamental matrix check // // 
            
            //     vector<cv::Point2f> un_l_pts, un_r_pts; 
            //     un_l_pts.reserve(ids.size()); 
            //     un_r_pts.reserve(ids.size()); 

            //     for(int i=0; i<ids.size(); i++){
            //         if(status[i]){
            //             Eigen::Vector3d tmp_p;
            //             stereo_m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);

            //             un_l_pts.push_back(cv::Point2f(tmp_p.x(), tmp_p.y())); 


            //             stereo_m_camera[1]->liftProjective(Eigen::Vector2d(cur_right_pts[i].x, cur_right_pts[i].y), tmp_p);

            //             un_r_pts.push_back(cv::Point2f(tmp_p.x(), tmp_p.y()));
            //         }
            //     }


            // ids_right = ids;
            // reduceVector(cur_right_pts, status);
            // reduceVector(ids_right, status);
            // reduceVector(track_cnt_right, status);

            // assert(cur_right_pts.size() == un_l_pts.size()); 
            // vector<uchar> lr_fund_status(cur_right_pts.size(), 0); 

            // int cnt_right_inlier = 0; 
            // for(int i=0; i<lr_fund_status.size(); i++){
                
            //     // check epipilar distance 
            //     Vector3d p3d0(un_l_pts[i].x, un_l_pts[i].y, 1.);
            //     Vector3d p3d1(un_r_pts[i].x, un_r_pts[i].y, 1.);  
            //     const double epipolar_error =
            //     std::abs(p3d1.transpose() * Eeesntial_matrix_event * p3d0);
            //     if(epipolar_error < 0.005) // epipolar distance 
            //     {
            //         lr_fund_status[i] = 1; 
            //         ++cnt_right_inlier;
            //     }
            // }

            // reduceVector(cur_right_pts, lr_fund_status);
            // reduceVector(ids_right, lr_fund_status);

            // // fundamental matrix check // // 


            for (auto &n : track_cnt_right)
                n++;

            for (auto &p : cur_right_pts)
            {
                track_cnt_right.push_back(1);
            }

            cur_un_right_pts = undistortedPts(cur_right_pts, stereo_m_camera[1]);
            right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);

        }
        prev_un_right_pts_map = cur_un_right_pts_map;
    }

    curLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        curLeftPtsMap[ids[i]] = cur_pts[i];

    curRightPtsMap.clear();
    for(size_t i = 0; i < cur_right_pts.size(); i++)
        curRightPtsMap[ids_right[i]] = cur_right_pts[i];

    prev_img_left = cur_img_left;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;
    prev_event_mat_left = event_mat_left;

    if(SHOW_TRACK){
        stereo_event_drawTrack(event_mat_left, event_mat_right, ids, cur_pts, cur_right_pts, prevLeftPtsMap);
        event_drawTrack_two(cur_img_left, prev_img_left, ids, cur_pts, prev_pts, prevLeftPtsMap);//tracking
        event_drawTrack_stereo(cur_img_left, cur_img_right, prev_img_left, ids, ids_right, cur_pts, cur_right_pts, curLeftPtsMap, curRightPtsMap);//matching
    }

    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];

    return;
}

void FeatureTracker::trackEvent(double _cur_time, const dvs_msgs::EventArray &event_left, const dvs_msgs::EventArray &event_right, const Motion_correction_value measurements)
{
    TicToc t_r;
    cur_time = _cur_time;
    cv::Mat img_left;
    cv::Mat img_right;
    cv::Mat rightImg;
    double dt;

    if(FLAG_DETECTOR_NOSTART){
        FLAG_DETECTOR_NOSTART = false;
        detector.init(COL_event,ROW_event,fx,fy,cx,cy);
    }

    Motion_correction_value correct_time_measurement = measurements;
    correct_time_measurement.second.second.first[0] = event_left.events[0].ts.toSec();
    correct_time_measurement.second.second.first[1] = event_left.header.stamp.toSec();
    dt = correct_time_measurement.second.second.first[1] - correct_time_measurement.second.second.first[0];

    detector.cur_event_mat_left = cv::Mat::zeros(cv::Size(COL_event, ROW_event), CV_8UC3);
    detector.cur_event_mat_right = cv::Mat::zeros(cv::Size(COL_event, ROW_event), CV_8UC3);
    
    for (const dvs_msgs::Event& e_left:event_left.events){
        if (dt > 0 && (e_left.ts.toSec() - correct_time_measurement.second.second.first[0])/dt < 1){
            detector.createSAE_left(e_left.ts.toSec(), e_left.x, e_left.y, e_left.polarity, correct_time_measurement);
        }else{
            detector.createSAE_left(e_left.ts.toSec(), e_left.x, e_left.y, e_left.polarity);
        }
    }

    for (const dvs_msgs::Event& e_right:event_right.events){
        if (dt > 0 && (e_right.ts.toSec() - correct_time_measurement.second.second.first[0])/dt < 1){
            detector.createSAE_right(e_right.ts.toSec(), e_right.x, e_right.y, e_right.polarity, correct_time_measurement);
        }else{
            detector.createSAE_right(e_right.ts.toSec(), e_right.x, e_right.y, e_right.polarity);
        }
    }

    cv::Mat event_mat_left = detector.cur_event_mat_left;
    cv::Mat event_mat_right = detector.cur_event_mat_right;

    const cv::Mat time_surface_map_left = detector.SAEtoTimeSurface_left(cur_time);
    const cv::Mat time_surface_map_right = detector.SAEtoTimeSurface_right(cur_time);

    time_surface_visualization_left = time_surface_map_left.clone();
    time_surface_visualization_right = time_surface_map_right.clone();
    cv::Mat time_surface_left = time_surface_map_left.clone();
    cv::Mat time_surface_right = time_surface_map_right.clone();
    
    if (EQUALIZE)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->apply(time_surface_left,img_left);
        clahe->apply(time_surface_right,img_right);
        cv::normalize(img_left, img_left, 0, 255, CV_MINMAX);
        cv::normalize(img_right, img_right, 0, 255, CV_MINMAX);
    }
    else
    {
        img_left = time_surface_left;
        img_right = time_surface_right;
    }


    if(cur_img_left.empty()){
        prev_img_left = cur_img_left = img_left;
    }else{
        cur_img_left = img_left; 
    }
    cur_pts.clear();

    if(cur_img_right.empty()){
        prev_img_right = cur_img_right =img_right;
    }else{
        cur_img_right = img_right;
    }
    cur_right_pts.clear();
    
    TicToc t_temporal;
    if (prev_pts.size() > 0)
    {   
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        cv::calcOpticalFlowPyrLK(prev_img_left, cur_img_left, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);

        // reverse check
        if(FLOW_BACK)
        {
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts;
            cv::calcOpticalFlowPyrLK(cur_img_left, prev_img_left, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);

            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }
        
        for (int i = 0; i < int(cur_pts.size()); i++)
            if (status[i] && !inBorder_event(cur_pts[i]))
                status[i] = 0;
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(ids, status );
        reduceVector(track_cnt, status);

    }

    for (auto &n : track_cnt)
        n++;

    if (PUB_THIS_FRAME)
    {
        rejectWithF_event();
        TicToc t_m;
        Event_setMask();

        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
        if (n_max_cnt > 0)
        {   
            if(mask_event.empty())
                cout << "mask is empty " << endl;
            if (mask_event.type() != CV_64FC1)
                cout << "mask type wrong " << endl;
            if (mask_event.size() != time_surface_left.size())
                cout << "wrong size" << endl;
            
            Event_FeaturesToTrack( event_left, n_pts, cur_pts, MAX_CNT - cur_pts.size(), MIN_DIST, mask_event, time_surface_left);

        }
        else
            n_pts.clear();

        for (auto &p : n_pts)
        {
            cur_pts.push_back(p);
            ids.push_back(n_id++);
            track_cnt.push_back(1);
        }
    }
    cur_un_pts.clear();
    cur_un_pts = undistortedPts(cur_pts, stereo_m_camera[0]);
    pts_velocity.clear();
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);
    
    TicToc t_spatial;
    //////// Event Stereo ////////

    if(!img_right.empty())
    {
        ids_right.clear();
        cur_right_pts.clear();
        cur_un_right_pts.clear();
        right_pts_velocity.clear();
        cur_un_right_pts_map.clear();
        if(!cur_pts.empty())
        {
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            vector<float> err;
            // cur left ---- cur right matching
            cv::calcOpticalFlowPyrLK(cur_img_left, cur_img_right, cur_pts, cur_right_pts, status, err, cv::Size(21, 21), 3);
      
            // // reverse check cur right ---- cur left matching
            if(FLOW_BACK && !cur_right_pts.empty())
            {
                cv::calcOpticalFlowPyrLK(cur_img_right, cur_img_left, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
                for(size_t i = 0; i < status.size(); i++)
                {
                    if(status[i] && statusRightLeft[i] && inBorder_event(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
                    {
                        status[i] = 1;
                    }
                    else
                        status[i] = 0;
                }
            }

            //fundamental matrix check 
            
            //     vector<cv::Point2f> un_l_pts, un_r_pts; 
            //     un_l_pts.reserve(ids.size()); 
            //     un_r_pts.reserve(ids.size()); 

            //     for(int i=0; i<ids.size(); i++){
            //         if(status[i]){
            //             Eigen::Vector3d tmp_p;
            //             stereo_m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            //             // ouf2 << i<<" l: "<< tmp_p.transpose()<<" "; 
            //             // tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            //             // tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            //             un_l_pts.push_back(cv::Point2f(tmp_p.x(), tmp_p.y())); 
            //             // ouf2<<" proj: "<<tmp_p.x()<<" "<<tmp_p.y()<<" "<<endl; 

            //             stereo_m_camera[1]->liftProjective(Eigen::Vector2d(cur_right_pts[i].x, cur_right_pts[i].y), tmp_p);
            //             // ouf2 << i<<" r: "<< tmp_p.transpose()<<" "; 
            //             // tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            //             // tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            //             un_r_pts.push_back(cv::Point2f(tmp_p.x(), tmp_p.y()));
            //             // ouf2<<" proj: "<<tmp_p.x()<<" "<<tmp_p.y()<<" "<<endl; 
            //             // ouf<<i<<" "<<cur_pts[i].x<<" "<<cur_pts[i].y<<" "<<cur_right_pts[i].x<<" "<<cur_right_pts[i].y<<endl;
            //         }
            //     }

            // // use Tlr to to verification 

            ids_right = ids;
            reduceVector(cur_right_pts, status);
            reduceVector(ids_right, status);
            reduceVector(track_cnt_right, status);

            for (auto &n : track_cnt_right)
                n++;

            // assert(cur_right_pts.size() == un_l_pts.size()); 
            // vector<uchar> lr_fund_status(cur_right_pts.size(), 0); 

            // int cnt_right_inlier = 0; 
            // for(int i=0; i<lr_fund_status.size(); i++){
                
            //     // check epipilar distance 
            //     Vector3d p3d0(un_l_pts[i].x, un_l_pts[i].y, 1.);
            //     Vector3d p3d1(un_r_pts[i].x, un_r_pts[i].y, 1.);  
            //     const double epipolar_error =
            //     std::abs(p3d1.transpose() * Eeesntial_matrix_event * p3d0);//Essential_matrix is in the parameters.cpp, calculated by Trl & Rrl
            //     if(epipolar_error < 0.005) // epipolar distance 
            //     {
            //         lr_fund_status[i] = 1; 
            //         ++cnt_right_inlier;
            //     }
            // }

            // reduceVector(cur_right_pts, lr_fund_status);
            // reduceVector(ids_right, lr_fund_status);
            // reduceVector(track_cnt_right, lr_fund_status);

            for (auto &p : cur_right_pts)
            {
                track_cnt_right.push_back(1);
            }

            cur_un_right_pts = undistortedPts(cur_right_pts, stereo_m_camera[1]);
            right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
        }
        prev_un_right_pts_map = cur_un_right_pts_map;
    }

    //////// Event Stereo ////////

    if(SHOW_TRACK){
        stereo_event_drawTrack(event_mat_left, event_mat_right, ids, cur_pts, cur_right_pts, prevLeftPtsMap);
        stereo_event_loop(event_mat_left);
        event_drawTrack_two(cur_img_left, prev_img_left, ids, cur_pts, prev_pts, prevLeftPtsMap);
    }
    prev_img_left = cur_img_left;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;

    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];

    return;
}

cv::Mat FeatureTracker::getTrackImage()
{   
    return imTrack;
}

cv::Mat FeatureTracker::getLoopImage()
{
    return Image_loop;
}

cv::Mat FeatureTracker::getTrackImage_two()
{
    return imTrack_two;
}

cv::Mat FeatureTracker::gettimesurface()
{
    return time_surface_visualization_left;
}

cv::Mat FeatureTracker::getTrackImage_two_point()
{
    return imTrack_two_point;
}

cv::Mat FeatureTracker::getEventloop()
{
    return imTrack_loop;
}

// F RANSAC
void FeatureTracker::rejectWithF_event()// left cur, left pre
{
    if (cur_pts.size() >= 8)// at least 8 points
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_prev_pts(prev_pts.size());
        for (unsigned int i = 0; i < prev_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            // process the previous point and normalize it
            stereo_m_camera[0]->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);// pixel coordinate to nomalized coordinate, tmp_p is the normalized point
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL_event / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW_event / 2.0;
            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            //process the current point and normalize it
            stereo_m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL_event / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW_event / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        // F RANSAC
        cv::findFundamentalMat(un_prev_pts, un_cur_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = prev_pts.size();//prev points size
        // remove left outliers
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
        // ROS_INFO("FM ransac for the left point: %d -> %lu: %f%", size_a, cur_pts.size(), 100.0*cur_pts.size()/size_a);

    }
}


bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}


void FeatureTracker::stereo_readIntrinsicParameter(vector<string>& calib_file)
{
    for(int i=0; i<calib_file.size(); i++){
        ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
        camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
        stereo_m_camera.push_back(camera);

        K_= camera->initUndistortRectifyMap(undist_map1_,undist_map2_);    
        fx = K_.at<float>(0, 0);
        fy = K_.at<float>(1, 1);
        cx = K_.at<float>(0, 2);
        cy = K_.at<float>(1, 2);
    }
}

cv::Point2f FeatureTracker::undistortedPts(cv::Point2f &pts, camodocal::CameraPtr cam)
{
    cv::Point2f un_pts;

    Eigen::Vector2d a(pts.x, pts.y);
    Eigen::Vector3d b;
    cam->liftProjective(a, b);
    un_pts=cv::Point2f(b.x() / b.z(), b.y() / b.z());

    return un_pts;
}


vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam)
{
    vector<cv::Point2f> un_pts;
    for (unsigned int i = 0; i < pts.size(); i++)
    {
        Eigen::Vector2d a(pts[i].x, pts[i].y);
        Eigen::Vector3d b;
        cam->liftProjective(a, b);
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    }
    return un_pts;
}

vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                            map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts)
{
    vector<cv::Point2f> pts_velocity;
    cur_id_pts.clear();
    for (unsigned int i = 0; i < ids.size(); i++)
    {
        cur_id_pts.insert(make_pair(ids[i], pts[i]));
    }

    // caculate points velocity
    if (!prev_id_pts.empty())
    {
        double dt = cur_time - prev_time;//time check
        
        for (unsigned int i = 0; i < pts.size(); i++)
        {
            if(ids[i]!=-1){
                std::map<int, cv::Point2f>::iterator it;
                it = prev_id_pts.find(ids[i]);
                if (it != prev_id_pts.end())
                {
                    double v_x = (pts[i].x - it->second.x) / dt;
                    double v_y = (pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }else{
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    return pts_velocity;
}

/*                 draw function                */

void  FeatureTracker::event_drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap)
{
    int rows = imLeft.rows;
    int cols = imLeft.cols;

    if (!imRight.empty()){
        cv::hconcat(imLeft, imRight, imTrack);//horizontally concatenate two images
    }
    else{
        imTrack = imLeft.clone();
    }

    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        if(track_cnt[j]>=2)
            cv::circle(imTrack, curLeftPts[j], 3, cv::Scalar(0, 0, 255),  -1);
        else
            cv::circle(imTrack, curLeftPts[j], 3, cv::Scalar(0, 255, 0),  1);       
    }

    map<int, cv::Point2f>::iterator mapIt;
    for (size_t i = 0; i < curLeftIds.size(); i++)
    {   
        int id = curLeftIds[i];
        mapIt = prevLeftPtsMap.find(id);
        if(mapIt != prevLeftPtsMap.end())
        {
            if(track_cnt[i]>=2){

                Vector2d tmp_cur_un_pts (cur_un_pts[i].x, cur_un_pts[i].y);
                Vector2d tmp_pts_velocity (pts_velocity[i].x, pts_velocity[i].y);
                Vector3d tmp_prev_un_pts;
                tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                tmp_prev_un_pts.z() = 1;
                Vector2d tmp_prev_uv;
                m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                cv::arrowedLine(imTrack, curLeftPts[i], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(0, 255, 0), 2 , 8, 0);

            }
        }
    }
}


void  FeatureTracker::stereo_event_drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap)
{
    int rows = imLeft.rows;
    int cols = imLeft.cols;

    if (!imRight.empty()){
        cv::hconcat(imLeft, imRight, imTrack);
    }
    else{
        imTrack = imLeft.clone();
    }

    for (size_t j = 0; j < curLeftPts.size(); j++)
    {   
        if(track_cnt[j]>=2)
            cv::circle(imTrack, curLeftPts[j], 3, cv::Scalar(0, 255, 0),  -1);
        else
            cv::circle(imTrack, curLeftPts[j], 3, cv::Scalar(0, 0, 255),  -1);
    }

    if (!imRight.empty()){
        for (size_t i = 0; i < curRightPts.size(); i++){
            cv::Point2f rightPt = curRightPts[i];
            rightPt.x += cols;

        if (track_cnt_right[i]>=2)
            cv::circle(imTrack, rightPt, 3, cv::Scalar(0, 255, 0), -1);//bgr
        else
            cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
        }
    }

    map<int, cv::Point2f>::iterator mapIt;
    for (size_t i = 0; i < curLeftIds.size(); i++)
    {   
        int id = curLeftIds[i];
        mapIt = prevLeftPtsMap.find(id);
        if(mapIt != prevLeftPtsMap.end())
        {
            if(track_cnt[i]>=15){
                Vector2d tmp_cur_un_pts (cur_un_pts[i].x, cur_un_pts[i].y);
                Vector2d tmp_pts_velocity (pts_velocity[i].x, pts_velocity[i].y);
                Vector3d tmp_prev_un_pts;
                tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                tmp_prev_un_pts.z() = 1;
                Vector2d tmp_prev_uv;
                stereo_m_camera[0]->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);//
                cv::arrowedLine(imTrack, curLeftPts[i], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(0, 255, 0), 2 , 8, 0);
            }
        }
    }

}


void  FeatureTracker::stereo_event_loop(const cv::Mat &imLeft)
{
    int rows = imLeft.rows;
    int cols = imLeft.cols;

    imTrack_loop = imLeft.clone();
}


void  FeatureTracker::event_drawTrack_two(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap)
{
    int rows = imLeft.rows;
    int cols = imLeft.cols;
    if (!imRight.empty())
        cv::vconcat(imLeft, imRight, imTrack_two);
    else
        imTrack_two = imLeft.clone();
    cv::cvtColor(imTrack_two, imTrack_two, CV_GRAY2RGB);

    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        if(track_cnt[j]>=2)
            cv::circle(imTrack_two, curLeftPts[j], 3, cv::Scalar(0, 0, 255), -1);
    }
    
    if (!imRight.empty())
    {
        map<int, cv::Point2f>::iterator mapIt;
        for (size_t i = 0; i < curLeftIds.size(); i++)
        {
            int id = curLeftIds[i];
            mapIt = prevLeftPtsMap.find(id);
            cv::Point2f rightPt = mapIt->second;
            rightPt.y += rows;
            if(mapIt != prevLeftPtsMap.end()){
                cv::circle(imTrack_two, rightPt, 3, cv::Scalar(0, 255,255), -1);
                cv::Point2f leftPt = curLeftPts[i];
                if(track_cnt[i]>=2){
                    // cv::arrowedLine(imTrack_two, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0, 0.02);//绿色的
                }
            }
        }
    }
}

void  FeatureTracker::event_drawTrack_stereo(const cv::Mat &imLeft, const cv::Mat &imRight, const cv::Mat &PrevimLeft,
                               vector<int> &curLeftIds,
                               vector<int> &curRightIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &curLeftPtsMap,
                               map<int, cv::Point2f> &curRightPtsMap)
{
    int rows = imLeft.rows;
    int cols = imLeft.cols;
    if (!imRight.empty()){
        cv::hconcat(imLeft, imRight, imTrack_two_point);
        // cv::vconcat(imTrack_two_line, PrevimLeft, imTrack_two_line);
    }
    else
        imTrack_two_point = imLeft.clone();
    cv::cvtColor(imTrack_two_point, imTrack_two_point, CV_GRAY2RGB);

    map<int, cv::Point2f>::iterator mapIt_right;
    for (size_t j = 0; j < curRightPts.size(); j++)
    {
        int id = curRightIds[j];
        mapIt_right = curRightPtsMap.find(id);
        cv::Point2f rightPt = mapIt_right->second;
        rightPt.x += cols;
        if(track_cnt_right[j]>=2)
            cv::circle(imTrack_two_point, rightPt, 3, cv::Scalar(255, 0, 0), -1);
        else
            cv::circle(imTrack_two_point, rightPt, 3, cv::Scalar(255, 0, 0), -1);
    }
    
    if (!imLeft.empty())
    {
        map<int, cv::Point2f>::iterator mapIt;
        for (size_t i = 0; i < curRightIds.size(); i++)
        {
            int id = curRightIds[i];
            mapIt = curLeftPtsMap.find(id);
            cv::Point2f leftPt = mapIt->second;
            if(mapIt != curLeftPtsMap.end()){
                cv::circle(imTrack_two_point, leftPt, 3, cv::Scalar(0, 0,255), -1);
                cv::Point2f rightPt = curRightPts[i];
                rightPt.x += cols;
                // rightPt.y += rows;//vertical concatenate
                if(track_cnt[i]>=2){
                    cv::arrowedLine(imTrack_two_point, rightPt, leftPt, cv::Scalar(0, 255, 0), 1, 8, 0, 0.02);//绿色的
                }
            }
        }
    }

    
}


void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap)
{
    int cols = imLeft.cols;
    if (!imRight.empty())
        cv::hconcat(imLeft, imRight, imTrack);
    else
        imTrack = imLeft.clone();
    cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);

    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        if (track_cnt[j]>=15)
            cv::circle(imTrack, curLeftPts[j], 3, cv::Scalar(255, 0, 0), -1);//bgr
        else if (track_cnt[j]>=2)
            cv::circle(imTrack, curLeftPts[j], 3, cv::Scalar(0, 255, 0), -1);//bgr
        else
            cv::circle(imTrack, curLeftPts[j], 3, cv::Scalar(0, 0, 255), -1);
    }
    if (!imRight.empty())
    {
        for (size_t i = 0; i < curRightPts.size(); i++)
        {
            cv::Point2f rightPt = curRightPts[i];
            rightPt.x += cols;
        
        if (track_cnt_right[i]>=15)
            cv::circle(imTrack, rightPt, 3, cv::Scalar(255, 0, 0), -1);//bgr
        else if (track_cnt_right[i]>=2)
            cv::circle(imTrack, rightPt, 3, cv::Scalar(0, 255, 0), -1);//bgr
        else
            cv::circle(imTrack, rightPt, 3, cv::Scalar(0, 0, 255), -1);

        }
    }
    
    map<int, cv::Point2f>::iterator mapIt;
    for (size_t i = 0; i < curLeftIds.size(); i++)
    {
        int id = curLeftIds[i];
        mapIt = prevLeftPtsMap.find(id);
        if(mapIt != prevLeftPtsMap.end())
        {
            if(track_cnt[i]>=2){
              cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
            }
        }
    }

}

double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2)
{
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

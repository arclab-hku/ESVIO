#include "feature_manager.h"


// triangulation to compute depth 
double Stereo_Event_FeaturePerFrame::getDepth()
{
    if(!is_stereo) return -1; 
    if(dpt > 0) return dpt; 
    if(point(0) < pointRight(0)){
        is_stereo = false; // this is false stereo match, since depth < 0 
        dpt = -1; 
        return -1; 
    }

    // check the triangulated depth 
    Eigen::Matrix<double, 3, 4> leftPose = Eigen::Matrix<double, 3, 4>::Zero();
    leftPose.leftCols<3>() = Eigen::Matrix3d::Identity(); 

    Eigen::Matrix<double, 3, 4> rightPose = Eigen::Matrix<double, 3, 4>::Zero(); 
    rightPose.leftCols<3>() = Rrl_event; 
    rightPose.rightCols<1>() = Trl_event; 

    Eigen::Vector2d point0, point1; 
    Eigen::Vector3d point3d; 
    point0 = point.head(2); 
    point1 = pointRight.head(2); 

    ((FeatureManager*)0)->Stereo_Event_triangulatePoint(leftPose, rightPose, point0, point1, point3d); 

    double depth = point3d.z(); 

    if(depth <= 1. || depth >= 7.){ // barely no object so close to the camera 
        is_stereo = false; 
        dpt = -1; 

        return -1; 
    }

    // check reprojection error 
    Eigen::Vector3d proj_pt0, proj_pt1; 
    proj_pt0 = point3d / depth; 
    proj_pt1 = Rrl_event * point3d + Trl_event; 
    if(proj_pt1.z()<= 1.){
        is_stereo = false; 
        dpt = -1; 
        return -1; 
    }
    proj_pt1 = proj_pt1/proj_pt1.z(); 

    Eigen::Vector2d err_pt0 = proj_pt0.head(2) - point0; 
    Eigen::Vector2d err_pt1 = proj_pt1.head(2) - point1; 

    double ep0_norm = err_pt0.norm(); 
    double ep1_norm = err_pt1.norm(); 

    if((err_pt0.norm() > 2./FOCAL_LENGTH) || (err_pt1.norm() > 2./FOCAL_LENGTH)){
        is_stereo = false; 
        dpt = -1; 
   
        return - 1; 
    }

    dpt = depth; 

    if(!g_use_stereo_correction)
        return dpt; 

    // stereo correction
    Vector3d np0 = point; 
    Vector3d np1 = pointRight; 

    Eigen::Matrix3d R = Rrl_event.transpose();
    Vector3d h = Trl_event;  
    Eigen::Matrix3d G = Utility::skewSymmetric(h)*R; 
    Eigen::Matrix3d Gt = G.transpose(); 
    Eigen::Matrix3d Pk = Eigen::Matrix3d::Identity(); 
    Pk(2,2) = 0; 

    double fe = np0.transpose() * G * np1; 
 
    Vector3d ve1 =  Pk*Gt*np0;
    Vector3d ve2 =  Pk*G*np1;
    double de = ve1.squaredNorm() + ve2.squaredNorm();
    Vector3d delta_np0 = fe * Pk * G * np1; 
    Vector3d delta_np1 = fe * Pk * Gt * np0; 


    Vector3d new_p0 = np0 - delta_np0/de; 
    Vector3d new_p1 = np1 - delta_np1/de; 

    point0 = new_p0.head(2); 
    point1 = new_p1.head(2); 

    ((FeatureManager*)0)->Stereo_Event_triangulatePoint(leftPose, rightPose, point0, point1, point3d); 
    
    // check reprojection error 
    proj_pt0 = point3d / point3d.z(); 
    proj_pt1 = Rrl_event * point3d + Trl_event; 
    if(proj_pt1.z()<= 1.){
        is_stereo = false; 
        dpt = -1; 
        return -1; 
    }
    proj_pt1 = proj_pt1/proj_pt1.z(); 

    err_pt0 = proj_pt0.head(2) - point0; 
    err_pt1 = proj_pt1.head(2) - point1; 
    double new_ep0_norm = err_pt0.norm();
    double new_ep1_norm = err_pt1.norm(); 
    if(new_ep0_norm > ep0_norm || new_ep1_norm > ep1_norm){
        gc_succeed = false; 
        return depth ; 
    }

    point = new_p0; 
    pointRight = new_p1; 
    depth = point3d.z(); 
    dpt = depth; 
    gc_succeed = true; 
    return depth; 
}

// triangulation to compute depth 
double stereo_FeaturePerFrame::getDepth()
{
    if(!is_stereo) return -1; 
    if(dpt > 0) return dpt; 
    if(point(0) < pointRight(0)){
        is_stereo = false; // this is false stereo match, since depth < 0 
        dpt = -1; 
        return -1; 
    }

    // check the triangulated depth 
    Eigen::Matrix<double, 3, 4> leftPose = Eigen::Matrix<double, 3, 4>::Zero();
    leftPose.leftCols<3>() = Eigen::Matrix3d::Identity(); 

    Eigen::Matrix<double, 3, 4> rightPose = Eigen::Matrix<double, 3, 4>::Zero(); 
    rightPose.leftCols<3>() = Rrl; 
    rightPose.rightCols<1>() = Trl; 

    Eigen::Vector2d point0, point1; 
    Eigen::Vector3d point3d; 
    point0 = point.head(2); 
    point1 = pointRight.head(2); 

    ((FeatureManager*)0)->triangulatePoint(leftPose, rightPose, point0, point1, point3d); 

    double depth = point3d.z(); 
    if(depth <= 1. || depth >= 7.){
        is_stereo = false; 
        dpt = -1; 

        return -1; 
    }

    Eigen::Vector3d proj_pt0, proj_pt1; 
    proj_pt0 = point3d / depth; 
    proj_pt1 = Rrl * point3d + Trl; 
    if(proj_pt1.z()<= 1.){
        is_stereo = false; 
        dpt = -1; 
        return -1; 
    }
    proj_pt1 = proj_pt1/proj_pt1.z(); 

    Eigen::Vector2d err_pt0 = proj_pt0.head(2) - point0; 
    Eigen::Vector2d err_pt1 = proj_pt1.head(2) - point1; 

    double ep0_norm = err_pt0.norm(); 
    double ep1_norm = err_pt1.norm(); 

    if((err_pt0.norm() > 2./FOCAL_LENGTH) || (err_pt1.norm() > 2./FOCAL_LENGTH)){
        is_stereo = false; 
        dpt = -1; 
   
        return - 1; 
    }

    dpt = depth; 

    if(!g_use_stereo_correction)  
        return dpt; 


    Vector3d np0 = point; 
    Vector3d np1 = pointRight; 

    Eigen::Matrix3d R = Rrl.transpose();
    Vector3d h = Trl;  
    Eigen::Matrix3d G = Utility::skewSymmetric(h)*R; 
    Eigen::Matrix3d Gt = G.transpose(); 
    Eigen::Matrix3d Pk = Eigen::Matrix3d::Identity(); 
    Pk(2,2) = 0; 

    double fe = np0.transpose() * G * np1; 
    Vector3d ve1 =  Pk*Gt*np0;
    Vector3d ve2 =  Pk*G*np1;
    double de = ve1.squaredNorm() + ve2.squaredNorm();
    Vector3d delta_np0 = fe * Pk * G * np1; 
    Vector3d delta_np1 = fe * Pk * Gt * np0; 

    Vector3d new_p0 = np0 - delta_np0/de; 
    Vector3d new_p1 = np1 - delta_np1/de; 

    point0 = new_p0.head(2); 
    point1 = new_p1.head(2); 

    ((FeatureManager*)0)->triangulatePoint(leftPose, rightPose, point0, point1, point3d); 
    
    // check reprojection error 
    proj_pt0 = point3d / point3d.z(); 
    proj_pt1 = Rrl * point3d + Trl; 
    if(proj_pt1.z()<= 1.){
        is_stereo = false; 
        dpt = -1; 
        return -1; 
    }
    proj_pt1 = proj_pt1/proj_pt1.z(); 

    err_pt0 = proj_pt0.head(2) - point0; 
    err_pt1 = proj_pt1.head(2) - point1; 
    double new_ep0_norm = err_pt0.norm();
    double new_ep1_norm = err_pt1.norm(); 
    if(new_ep0_norm > ep0_norm || new_ep1_norm > ep1_norm){
        gc_succeed = false; 
        return depth ; 
    }

    point = new_p0; 
    pointRight = new_p1; 
    depth = point3d.z(); 
    dpt = depth; 
    gc_succeed = true; 
    return depth; 
}

int stereo_FeaturePerId::endFrame()
{
    return start_frame + feature_per_frame.size() - 1;
}


int Stereo_Event_FeaturePerId::endFrame()
{
    return start_frame + Event_feature_per_frame.size() - 1;
}


//*********************************************************************************************************
FeatureManager::FeatureManager(Matrix3d _Rs[])
    : Rs(_Rs)
{
    for (int i = 0; i < NUM_OF_CAM * 2; i++)
        ric[i].setIdentity();
}

void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM * 2; i++)
    {
        ric[i] = _ric[i];
    }
}

void FeatureManager::Event_setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}

void FeatureManager::clearState()
{
    stereo_feature.clear();
    Stereo_Event_feature.clear();
}

int FeatureManager::Stereo_getImageFeatureCount()
{
    int cnt = 0;
    for (auto &it : stereo_feature)
    {

        it.used_num = it.feature_per_frame.size();

        if (it.used_num >= 2 && it.start_frame < WINDOW_SIZE - 2)
        {
            cnt++;
        }
    }
    return cnt;
}


int FeatureManager::Stereo_getEventFeatureCount()
{
    int cnt = 0;
    for (auto &it : Stereo_Event_feature)
    {

        it.used_num = it.Event_feature_per_frame.size();

        if (it.used_num >= 2 && it.start_frame < WINDOW_SIZE - 2)
        {
            cnt++;
        }
    }
    return cnt;
}


bool FeatureManager::stereo_addFeatureCheckParallax(int frame_count, 
                                             const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
                                             const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &event,  
                                             double td)
{   
    // ROS_DEBUG("num of image feature: %d", Stereo_getImageFeatureCount());
    // ROS_DEBUG("num of event feature: %d", Stereo_getEventFeatureCount());
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    last_average_parallax = 0;
    long_track_num = 0; 
    new_feature_num = 0;

    // event feature
    for (auto &id_pts : event)
    {
        Stereo_Event_FeaturePerFrame f_per_fra(id_pts.second[0].second, td);
        f_per_fra.feat_id = id_pts.first; 
        if(id_pts.second[0].first != 0)
            ROS_ERROR("what event? second[0].first = %d", id_pts.second[0].first );
        assert(id_pts.second[0].first == 0);
        if(id_pts.second.size() == 2)
        {
            f_per_fra.rightObservation(id_pts.second[1].second);
            assert(id_pts.second[1].first == 1);
        }

        int feature_id = id_pts.first;
        auto it = find_if(Stereo_Event_feature.begin(), Stereo_Event_feature.end(), [feature_id](const Stereo_Event_FeaturePerId &it)
                          {
            return it.feature_id == feature_id;
                          });

        if (it == Stereo_Event_feature.end())
        {
            Stereo_Event_feature.push_back(Stereo_Event_FeaturePerId(feature_id, frame_count));
            Stereo_Event_feature.back().Event_feature_per_frame.push_back(f_per_fra);
            // new_feature_num++;
        }
        else if (it->feature_id == feature_id)
        {
            it->Event_feature_per_frame.push_back(f_per_fra);
            // last_track_num++;
            // if( it-> Event_feature_per_frame.size() >= 4)
            //     long_track_num++;
        }
        
    }

    // image feature
    for (auto &id_pts : image)
    {
        stereo_FeaturePerFrame f_per_fra(id_pts.second[0].second, td);
        f_per_fra.feat_id = id_pts.first; 
        if(id_pts.second[0].first != 0)
            ROS_ERROR("what? second[0].first = %d", id_pts.second[0].first );
        assert(id_pts.second[0].first == 0);
        if(id_pts.second.size() == 2)
        {
            f_per_fra.rightObservation(id_pts.second[1].second);
            assert(id_pts.second[1].first == 1);
        }

        int feature_id = id_pts.first;
        auto it = find_if(stereo_feature.begin(), stereo_feature.end(), [feature_id](const stereo_FeaturePerId &it)
                          {
            return it.feature_id == feature_id;
                          });

        if (it == stereo_feature.end())
        {
            stereo_feature.push_back(stereo_FeaturePerId(feature_id, frame_count));
            stereo_feature.back().feature_per_frame.push_back(f_per_fra);
            new_feature_num++;
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
            if( it-> feature_per_frame.size() >= 4)
                long_track_num++;
        }
        
    }


    if (frame_count < 2 || last_track_num < 20)
        return true;

    for (auto &it_per_id : stereo_feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }   

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        // last_average_parallax = parallax_sum / parallax_num * FOCAL_LENGTH;
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

bool FeatureManager::ESIO_addFeatureCheckParallax(int frame_count,
                                             const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &event, 
                                             double td)
{   
    // ROS_DEBUG("input event features: %d ",(int)event.size());
    // ROS_DEBUG("num of event feature: %d", Stereo_getEventFeatureCount());
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    last_average_parallax = 0;
    long_track_num = 0; 
    new_feature_num = 0; 

    // event feature
    for (auto &id_pts : event)
    {
        Stereo_Event_FeaturePerFrame f_per_fra(id_pts.second[0].second, td);
        f_per_fra.feat_id = id_pts.first; 
        if(id_pts.second[0].first != 0)
            ROS_ERROR("what? second[0].first = %d", id_pts.second[0].first );
        assert(id_pts.second[0].first == 0);
        if(id_pts.second.size() == 2)
        {
            f_per_fra.rightObservation(id_pts.second[1].second);
            assert(id_pts.second[1].first == 1);
        }

        int feature_id = id_pts.first;
        auto it = find_if(Stereo_Event_feature.begin(), Stereo_Event_feature.end(), [feature_id](const Stereo_Event_FeaturePerId &it)
                          {
            return it.feature_id == feature_id;
                          });

        if (it == Stereo_Event_feature.end())
        {
            Stereo_Event_feature.push_back(Stereo_Event_FeaturePerId(feature_id, frame_count));
            Stereo_Event_feature.back().Event_feature_per_frame.push_back(f_per_fra);
            new_feature_num++;
        }
        else if (it->feature_id == feature_id)
        {
            it->Event_feature_per_frame.push_back(f_per_fra);
            last_track_num++;
            if( it-> Event_feature_per_frame.size() >= 4)
                long_track_num++;
        }
        
    }


    if (frame_count < 2 || last_track_num < 20)
        return true;

    for (auto &it_per_id : Stereo_Event_feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.Event_feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }   

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        // last_average_parallax = parallax_sum / parallax_num * FOCAL_LENGTH;
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : stereo_feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;

            b = it.feature_per_frame[idx_r].point;
            
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

vector<pair<Vector3d, Vector3d>> FeatureManager::Event_getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : Stereo_Event_feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.Event_feature_per_frame[idx_l].point;

            b = it.Event_feature_per_frame[idx_r].point;
            
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

vector<pair<Vector3d, Vector3d>> FeatureManager::getCorrespondingWithDepth(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : stereo_feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;
            a.z() = it.feature_per_frame[idx_l].getDepth(); 

            b = it.feature_per_frame[idx_r].point;
            b.z() = it.feature_per_frame[idx_r].getDepth();
            
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

vector<pair<Vector3d, Vector3d>> FeatureManager::Event_getCorrespondingWithDepth(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : Stereo_Event_feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.Event_feature_per_frame[idx_l].point;
            a.z() = it.Event_feature_per_frame[idx_l].getDepth(); 

            b = it.Event_feature_per_frame[idx_r].point;
            b.z() = it.Event_feature_per_frame[idx_r].getDepth();
            
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}


void FeatureManager::Stereo_Image_setDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : stereo_feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
    }
}

void FeatureManager::Stereo_Event_setDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : Stereo_Event_feature)
    {
        it_per_id.used_num = it_per_id.Event_feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
    }
}

void FeatureManager::removeFailures()
{

    for (auto it = stereo_feature.begin(), it_next = stereo_feature.begin();
    it != stereo_feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            stereo_feature.erase(it);
    }


    for (auto it = Stereo_Event_feature.begin(), it_next = Stereo_Event_feature.begin();
         it != Stereo_Event_feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            Stereo_Event_feature.erase(it);
    }
}


void FeatureManager::clearDepth(const VectorXd &x)
{
    int feature_index = -1;

    for (auto &it_per_id : stereo_feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth = 1.0 / x(++feature_index);
    }

}

void FeatureManager::Event_clearDepth(const VectorXd &x)
{
    int feature_index = -1;

    for (auto &it_per_id : Stereo_Event_feature)
    {
        it_per_id.used_num = it_per_id.Event_feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth = 1.0 / x(++feature_index);
    }
}


VectorXd FeatureManager::stereo_getDepthVector()
{
    VectorXd dep_vec(Stereo_getImageFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : stereo_feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
#if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    return dep_vec;
}


VectorXd FeatureManager::Stereo_Event_getDepthVector()
{
    VectorXd dep_vec(Stereo_getEventFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : Stereo_Event_feature)
    {
        it_per_id.used_num = it_per_id.Event_feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
#if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    return dep_vec;
}


void FeatureManager::triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    for (auto &it_per_id : stereo_feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        if (it_per_id.estimated_depth > 0)
            continue;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        // ROS_ASSERT(NUM_OF_CAM == 1);
        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;

            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        ROS_ASSERT(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];
        //it_per_id->estimated_depth = -b / A;
        //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;
        //it_per_id->estimated_depth = INIT_DEPTH;

        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
        }

    }
}

void FeatureManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Eigen::Vector4d triangulated_point;
    triangulated_point =
              design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

void FeatureManager::Stereo_Event_triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Eigen::Vector4d triangulated_point;
    triangulated_point =
              design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

void FeatureManager::triangulateStereo()
{
    for (auto &it_per_id : stereo_feature)
    {
        if (it_per_id.estimated_depth > 0)
            continue;
        double depth = it_per_id.feature_per_frame[0].getDepth();
        if(depth > 0){
            it_per_id.estimated_depth = depth;
        }
    }
}

void FeatureManager::stereo_triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    for (auto &it_per_id : stereo_feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        if (it_per_id.estimated_depth > 0)
            continue;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        // ROS_ASSERT(NUM_OF_CAM == 1);
        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;

            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        ROS_ASSERT(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];
        //it_per_id->estimated_depth = -b / A;
        //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;
        //it_per_id->estimated_depth = INIT_DEPTH;

        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
        }

    }
}


void FeatureManager::Event_triangulateStereo()
{
    for (auto &it_per_id : Stereo_Event_feature)
    {   
        // std::cout<<"Stereo Event Feature" << Stereo_Event_feature.size() << std::endl;
        if (it_per_id.estimated_depth > 0)
            continue;
        double depth = it_per_id.Event_feature_per_frame[0].getDepth();  
        if(depth > 0){
            it_per_id.estimated_depth = depth;
        }
    }
}

void FeatureManager::Stereo_Event_triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    for (auto &it_per_id : Stereo_Event_feature)
    {
        it_per_id.used_num = it_per_id.Event_feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        if (it_per_id.estimated_depth > 0)
            continue;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Eigen::MatrixXd svd_A(2 * it_per_id.Event_feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[1];
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[1];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        for (auto &it_per_frame : it_per_id.Event_feature_per_frame)
        {
            imu_j++;

            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[1];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[1];
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        ROS_ASSERT(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];
        //it_per_id->estimated_depth = -b / A;
        //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;
        //it_per_id->estimated_depth = INIT_DEPTH;

        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
        }

    }
}



void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{

    //stereo image
    for (auto it = stereo_feature.begin(), it_next = stereo_feature.begin();
        it != stereo_feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;  
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2)
            {
                stereo_feature.erase(it);
                continue;
            }
            else
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }
    }

    //stereo event
    for (auto it = Stereo_Event_feature.begin(), it_next = Stereo_Event_feature.begin();
         it != Stereo_Event_feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            Eigen::Vector3d uv_i = it->Event_feature_per_frame[0].point;  
            it->Event_feature_per_frame.erase(it->Event_feature_per_frame.begin());
            if (it->Event_feature_per_frame.size() < 2)
            {
                Stereo_Event_feature.erase(it);
                continue;
            }
            else
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }
    }


}

void FeatureManager::removeBack()
{

    // stereo image
    for (auto it = stereo_feature.begin(), it_next = stereo_feature.begin();
        it != stereo_feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
                stereo_feature.erase(it);
        }
    }

    // stereo event
    for (auto it = Stereo_Event_feature.begin(), it_next = Stereo_Event_feature.begin();
         it != Stereo_Event_feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->Event_feature_per_frame.erase(it->Event_feature_per_frame.begin());
            if (it->Event_feature_per_frame.size() == 0)
                Stereo_Event_feature.erase(it);
        }
    }
    
}

void FeatureManager::removeFront(int frame_count)
{

    // for stereo image
    for (auto it = stereo_feature.begin(), it_next = stereo_feature.begin(); it != stereo_feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->feature_per_frame.size() == 0)
                stereo_feature.erase(it);
        }
    }

    // for stereo event
    for (auto it = Stereo_Event_feature.begin(), it_next = Stereo_Event_feature.begin(); it != Stereo_Event_feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)
                continue;
            it->Event_feature_per_frame.erase(it->Event_feature_per_frame.begin() + j);
            if (it->Event_feature_per_frame.size() == 0)
                Stereo_Event_feature.erase(it);
        }
    }

}


double FeatureManager::compensatedParallax2(const stereo_FeaturePerId &it_per_id, int frame_count)
{
    //check the second last frame is keyframe or not
    //parallax betwwen seconde last frame and third last frame
    const stereo_FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const stereo_FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp;

    //int r_i = frame_count - 2;
    //int r_j = frame_count - 1;
    //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}

double FeatureManager::compensatedParallax2(const Stereo_Event_FeaturePerId &it_per_id, int frame_count)
{
    //check the second last frame is keyframe or not
    //parallax betwwen seconde last frame and third last frame
    const Stereo_Event_FeaturePerFrame &frame_i = it_per_id.Event_feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const Stereo_Event_FeaturePerFrame &frame_j = it_per_id.Event_feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp;

    //int r_i = frame_count - 2;
    //int r_j = frame_count - 1;
    //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}
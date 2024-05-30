#! /bin/bash

gnome-terminal --tab -e 'bash -c "roscore;exec bash"'
sleep 3s

# #################********************* Run on ECMD Dataset ********************************##############
# gnome-terminal --tab -e 'bash -c "roslaunch esvio_estimator esvio_ecmd.launch;exec bash"'
# gnome-terminal --tab -e 'bash -c "roslaunch esvio_estimator compressed_to_raw.launch;exec bash"'

# gnome-terminal --window -e 'bash -c "rosbag play --pause -r 1.0 --clock /home/cpy/Datasets/ECMD/small_circle_day_strong_0627_first_circle_new.synced.merged.bag;exec bash"'


# #################********************* Run on MVSEC Datasets ********************************##############
# gnome-terminal --tab -e 'bash -c "roslaunch esvio_estimator esvio_mvsec_flying.launch;exec bash"'

# gnome-terminal --window -e 'bash -c "rosbag play --pause -r 1.0 --clock /home/cpy/Datasets/MVSEC/indoor_flying1_data.bag;exec bash"'


# #################********************* Run on DSEC Datasets ********************************##############
gnome-terminal --tab -e 'bash -c "roslaunch esvio_estimator esvio_DSEC.launch;exec bash"'
# gnome-terminal --tab -e 'bash -c "roslaunch esvio_estimator esio_DSEC.launch;exec bash"'

gnome-terminal --window -e 'bash -c "rosparam set use_sim_time true;rosbag play --pause --clock /home/cpy/Datasets/DSEC/dsec.merged.bag;exec bash"'


# #################********************* Run hku Dataset ********************************##############
# gnome-terminal --tab -e 'bash -c "roslaunch esvio_estimator stereo_davis_open.launch"'
# gnome-terminal --tab -e 'bash -c "roslaunch esvio_estimator esvio.launch;exec bash"'
# gnome-terminal --tab -e 'bash -c "roslaunch esvio_estimator esio.launch;exec bash"'

# gnome-terminal --window -e 'bash -c "rosbag play --clock --pause /home/cpy/Datasets/ESVIO/HKU_aggressive_translation.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play --clock --pause /home/cpy/Datasets/ESVIO/HKU_HDR_circle.bag;exec bash"'
 
# #################********************* Run on VECtor Dataset ********************************##############
# gnome-terminal --tab -e 'bash -c "roslaunch esvio_estimator esvio_VECtor.launch;exec bash"'
# gnome-terminal --tab -e 'bash -c "roslaunch esvio_estimator esvio_VECtor_small_scale.launch;exec bash"'

# gnome-terminal --window -e 'bash -c "rosbag play --pause -r 1.0 --clock /home/cpy/Datasets/VECtor/school_dolly1.synced.merged.bag;exec bash"'




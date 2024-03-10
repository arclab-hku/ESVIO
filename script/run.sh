#! /bin/bash

gnome-terminal --tab -e 'bash -c "roscore;exec bash"'
sleep 3s

# #################********************* Run on ECMD Dataset ********************************##############
# gnome-terminal --tab -e 'bash -c "roslaunch esvio_estimator esvio_standard_urbandvs.launch;exec bash"'
# gnome-terminal --tab -e 'bash -c "roslaunch esvio_estimator compressed_to_raw.launch;exec bash"'

# gnome-terminal --window -e 'bash -c "rosbag play --pause -r 1.0 --clock xx.bag;exec bash"'


# #################********************* Run on MVSEC Datasets ********************************##############
# gnome-terminal --tab -e 'bash -c "roslaunch esvio_estimator esvio_mvsec_flying.launch;exec bash"'

# gnome-terminal --window -e 'bash -c "rosbag play --pause -r 1.0 --clock xx.bag;exec bash"'


# #################********************* Run on DSEC Datasets ********************************##############
# gnome-terminal --tab -e 'bash -c "roslaunch esvio_estimator esvio_DSEC.launch;exec bash"'
# gnome-terminal --tab -e 'bash -c "roslaunch esvio_estimator esio_DSEC.launch;exec bash"'

# gnome-terminal --window -e 'bash -c "rosbag play --pause -r 1.0 --clock xx.bag;exec bash"'


# #################********************* Run hku Dataset ********************************##############
# gnome-terminal --tab -e 'bash -c "roslaunch esvio_estimator stereo_davis_open.launch"'
gnome-terminal --tab -e 'bash -c "roslaunch esvio_estimator esvio.launch;exec bash"'
# gnome-terminal --tab -e 'bash -c "roslaunch esvio_estimator esio.launch;exec bash"'

gnome-terminal --window -e 'bash -c "rosbag play --clock --pause /home/cpy/Datasets/ESVIO/HKU_aggressive_small_flip.bag;exec bash"'


# #################********************* Run on VECtor Dataset ********************************##############
# gnome-terminal --tab -e 'bash -c "roslaunch esvio_estimator esvio_VECtor.launch;exec bash"'
# gnome-terminal --tab -e 'bash -c "roslaunch esvio_estimator esvio_VECtor_small_scale.launch;exec bash"'

# gnome-terminal --window -e 'bash -c "rosbag play --pause -r 1.0 --clock xx.bag;exec bash"'




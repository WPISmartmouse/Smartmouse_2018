# MUST BE RUN FROM root of the project (Smartmouse_2017)
#source /usr/share/gazebo/setup.sh
source /usr/local/share/gazebo/setup.sh
export GAZEBO_PLUGIN_PATH=${PWD}/.sim_build/sim:$GAZEBO_PLUGIN_PATH
export LD_LIBRARY_PATH=${PWD}/.sim_build/sim:/usr/local/lib/$LD_LIBRARY_PATH
export GAZEBO_MODEL_PATH=${PWD}/sim/models

# gazebo -u --seed 0 --verbose sim/gzmaze.world

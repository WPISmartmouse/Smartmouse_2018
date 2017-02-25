source /usr/share/gazebo/setup.sh
export GAZEBO_PLUGIN_PATH=${PWD}/build/sim:$GAZEBO_PLUGIN_PATH
export LD_LIBRARY_PATH=${PWD}/build/sim:$LD_LIBRARY_PATH
export GAZEBO_MODEL_PATH=${PWD}/sim/models

# gazebo -u --verbose gzmaze.world

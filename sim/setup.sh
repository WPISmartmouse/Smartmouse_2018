source /usr/share/gazebo/setup.sh
export GAZEBO_PLUGIN_PATH=${PWD}/build/gzmaze:$GAZEBO_PLUGIN_PATH
export LD_LIBRARY_PATH=${PWD}/build/gzmaze:$LD_LIBRARY_PATH
export GAZEBO_MODEL_PATH=${PWD}/gzmaze/models

# gazebo -u --verbose gzmaze.world

export GAZEBO_PLUGIN_PATH=${PWD}/build/gzmaze:\
/usr/lib/x86_64-linux-gnu/gazebo-7/plugins:\
$GAZEBO_PLUGIN_PATH

export LD_LIBRARY_PATH=/usr/local/lib\
${PWD}/build/gzmaze:\
/usr/local/lib/x86_64-linux-gnu/gazebo-7/plugins:\
$LD_LIBRARY_PATH

export GAZEBO_MODEL_PATH=${PWD}/gzmaze/models

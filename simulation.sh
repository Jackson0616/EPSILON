#!/bin/bash 

core_dir=/opt/cache
[ ! -d ${core_dir} ] && mkdir -p ${core_dir}#!
ulimit -c unlimited
sudo sysctl -w kernel.core_pattern=${core_dir}/core.%e.%p

YOUR_WORKSPACE_PATH=~/EPSILON_WS
cd ${YOUR_WORKSPACE_PATH}
catkin_make
source_cmd="source ${YOUR_WORKSPACE_PATH}/devel/setup.bash"
${source_cmd}

gnome-terminal --tab --title="roscore" -- bash -c 'roscore'
sleep 5
gnome-terminal --tab --title="rviz" -- bash -c '\
    source ~/EPSILON_WS/devel/setup.bash;\
    roscd phy_simulator/rviz/;\
    rviz -d phy_simulator_planning.rviz;'
sleep 1

gnome-terminal --tab --title="planner" -- bash -c '\
    source ~/EPSILON_WS/devel/setup.bash;\
    roslaunch planning_integrated test_ssc_with_eudm_ros.launch;\
    roslaunch ai_agent_planner onlane_ai_agent.launch;'
sleep 1

gnome-terminal --tab --title="simulator" -- bash -c '\
    source ~/EPSILON_WS/devel/setup.bash;\
    roslaunch phy_simulator phy_simulator_planning.launch;'
sleep 1

gnome-terminal --tab --title="tools" -- bash -c '\
    source ~/EPSILON_WS/devel/setup.bash;\
    roscd aux_tools/src/;python3 terminal_server.py;'
sleep 1



# roscd phy_simulator/rviz/; rviz -d phy_simulator_planning.rviz
# roslaunch planning_integrated test_ssc_with_eudm_ros.launch
# roslaunch ai_agent_planner onlane_ai_agent.launch

# roslaunch phy_simulator phy_simulator_planning.launch

# roscd aux_tools/src/
# python terminal_server.py
#!/bin/bash

#time . ./dd_dfs.sh ../src/husky/husky_gazebo/worlds/ditch.world ../husky_predef_msgs/goal_rough_terrain2 combinatorial random
#time . ./dd_dfs.sh ../src/husky/husky_gazebo/worlds/clearpath_playpen.world ../husky_predef_msgs/goal_playpen model2model avg_trajectory
#time . ./dd_dfs.sh ../src/husky/husky_gazebo/worlds/ditch.world ../husky_predef_msgs/goal_ditch model2model avg_trajectory
#time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/rubble_loose2.world ../husky_predef_msgs/goal_rubble_loose2 model2model avg_trajectory
#time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/rough_terrain_gap_dynamic_min2.world ../husky_predef_msgs/goal_ditch model2model avg_trajectory
#time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/ditch.world ../husky_predef_msgs/goal_ditch model2model avg_trajectory
#time . ./dd_dfs.sh ../src/husky/husky_gazebo/worlds/clearpath_playpen.world ../husky_predef_msgs/goal_playpen trajectory_seg crash_proximal
#
### testing type stuff
#time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/rough_terrain_gap_dynamic_min3.world ../husky_predef_msgs/goal_ditch model2model avg_trajectory


# param setup
# feed in: navigation launch file(s), timeout
eval world_name=$(pwd)/$1
orig_world=$world_name
echo world name is $world_name
eval goalfilename=$(pwd)/$2
partitioning=$3
ordering=$4
goal_epsilon=$5
laser_enabled="true"
timeout=45
clusters=2

homedir="~"; eval homedir=$homedir
msg_dirname="$homedir/husky_ws/husky_predef_msgs"
script_dirname="$homedir/husky_ws/world_parser"
posefile="$homedir/husky_ws/src/husky/husky_navigation/scripts/poses.log"
cd ~/husky_ws/
source devel/setup.bash
# setup subworld directories
target_world_dir="$homedir/husky_ws/src/husky/husky_gazebo/worlds/subworlds"
rm $target_world_dir/*
orig_failure=$script_dirname/orig_failure.txt.backup
recent_failure=$script_dirname/recent_failure.txt
#cp $world_name $target_world_dir
#cp $world_name $target_world_dir/last_iter.world
subworld_count=2
total_generation_count=0
iteration_count=0
total_run_count=0
same_failures=0
diff_failures=0
flaky_test_reruns=0
goal_phrase="GOAL REACHED"
min_world_reached="false"
declare -a success_array

cp $world_name $target_world_dir/last_iter.world
models=$(python3 $script_dirname/model_counter.py $world_name 2>&1)
echo "Number of models in original world is" $models

for i in $(seq 1 3); do
   echo; echo; echo "RUN NUMBER" $i
   # run the OG world to gather data to characterize the failure
   . $script_dirname/execute_run.sh $world_name $goalfilename $timeout $partitioning
   retn_value=$?
   echo; echo "retn_value is" $retn_value
   if [[ $retn_value =~ "40" ]]; then
      echo "returned successful run; invalid starting world"
      return
   fi

   echo "goal_epsilon is " $goal_epsilon
   echo "Comparing trajectories..."
   result=$(python3 $script_dirname/compare_failures.py $orig_failure $posefile -e $goal_epsilon 2>&1)
   echo "Comparison of failures is " $result
   success_array[${#success_array[@]}]=$result
done
echo; echo; echo
for value in "${success_array[@]}"
do
     echo $value
done
cd world_parser
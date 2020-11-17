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
orig_failure=$script_dirname/orig_failure.txt
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
echo Number of models in original world is $models

# run the OG world to gather data to characterize the failure
. $script_dirname/execute_run.sh $world_name $goalfilename $timeout $partitioning
retn_value=$?
echo; echo return value for original world is $retn_value
if [[ $retn_value =~ "40" ]]; then
   echo "returned successful run; invalid starting world"
   return
fi
cp $posefile $orig_failure
echo "goal_epsilon is " $goal_epsilon
while [[ "${min_world_reached}" != "true" && $clusters -le $models ]]; do
      
   echo; echo BEGINNING OF LOOP
   models=$(python3 $script_dirname/model_counter.py $world_name 2>&1)
   echo Clusters: $clusters Models: $models
   echo; echo "Generating partitions..."
   echo "partitioning:" $partitioning "ordering: " $ordering "running script now.."
   # Get rid of old partitions
   find $target_world_dir -name '*[0-9].world' -delete
   find $target_world_dir -name '*_complement.world' -delete
   # partition the world & order the partitions
   # to partition -- may need the trajectory file, ros bag
   # to prioritize -- may need the crash data, trajectory file, ros bag
   if [[ $partitioning == 'combinatorial' ]]; then
      # combinatoric baseline
      python3 $script_dirname/partition_and_order.py $world_name --target_dir ${target_world_dir} -c
   elif [[ $partitioning == 'model2model' && $ordering == 'crash_proximal' ]]; then
      # model2model clustering and ordering by minimum distance from trajectory
      #python3 $script_dirname/partition_and_order.py $target_world_dir/last_iter.world --target_dir ${target_world_dir} --trajectory $posefile
      python3 $script_dirname/partition_and_order.py $world_name -d ${target_world_dir} -m2m $clusters -cd $posefile
   elif [[ $partitioning == 'model2model' && $ordering == 'avg_trajectory' ]]; then
      # model2model clustering and ordering by average distance from trajectory
      #python3 $script_dirname/partition_and_order.py $target_world_dir/last_iter.world --target_dir ${target_world_dir} --trajectory $posefile
      echo "GENERATING FILES NOW"
      python3 $script_dirname/partition_and_order.py $world_name -d ${target_world_dir} -m2m $clusters -atd $posefile
   elif [[ $partitioning == 'model2model' && $ordering == 'min_trajectory' ]]; then
      # model2model clustering and ordering by minimum distance from trajectory
      #python3 $script_dirname/partition_and_order.py $target_world_dir/last_iter.world --target_dir ${target_world_dir} --trajectory $posefile
      python3 $script_dirname/partition_and_order.py $world_name -d ${target_world_dir} -m2m $clusters -mtd $posefile
   elif [[ $partitioning == 'model2model' && $ordering == 'timestep' ]]; then
      #TODO: IMPLEMENT
      python3 $script_dirname/partition_and_order.py $world_name -d ${target_world_dir} -tseg $posefile -cd $posefile
   elif [[ $partitioning == 'trajectory_seg' && $ordering == 'crash_proximal' ]]; then
      #TODO: DEBUG
      python3 $script_dirname/partition_and_order.py $world_name -d ${target_world_dir} -tseg $posefile -cd $posefile -nsegs $clusters
   fi
   
   iteration_count=$(expr $iteration_count + 1)
   curr_generation_count=$(ls $target_world_dir | wc -l)
   if [[ -f $target_world_dir/"last_iter.world" ]]; then
      total_generation_count=$( expr $total_generation_count + $curr_generation_count - 1 )
      curr_generation_count=$( expr $curr_generation_count - 1 )
      echo "last_iter.world exists; total_generation_count is" $total_generation_count
   else
      total_generation_count=$( expr $total_generation_count + $curr_generation_count )
      echo "last_iter.world does not exist; total_generation_count is" $total_generation_count
   fi
   echo "total_generation_count is" $total_generation_count
   echo "curr_generation_count is" $curr_generation_count
   
   echo; echo "Selecting next world from partitions..."
   curr_successes=0
   curr_diff_failures=0
   for world in `ls $target_world_dir/* | sort -V`; do
      echo Choosing from...
      ls $target_world_dir/* | sort -V
      echo Checking world filename $world
      if [[ "$world" != *"last_iter.world" ]]; then 
         #world_name=$world
         echo; echo "Executing run using world" $world
         echo
         # run program on the new partitions
         
         . $script_dirname/execute_run.sh $world $goalfilename $timeout
         retn_value=$?
         echo; echo "retn_value is" $retn_value
         echo "ordering is " $ordering
         total_run_count=$(expr $total_run_count + 1)
         cp $posefile $recent_failure
         
         echo "Comparing trajectories..."
         echo "goal_epsilon is" $goal_epsilon
         result=$(python3 $script_dirname/compare_failures.py $orig_failure $recent_failure -e $goal_epsilon 2>&1)
         echo "Comparison of failures is " $result
         echo
         
         # SIMILAR UNSUCCESSFUL RUN --> partition again
         if [[ $retn_value =~ "50" && $result == "SAME" ]]; then
            # found similar failure --> repartition
            echo; echo "Found similar failure; repartitioning..."
            rm $target_world_dir/last_iter.world
            cp $world $target_world_dir/last_iter.world
            world_name=$target_world_dir/last_iter.world
            #rm $target_world_dir/.*-[0-9]*.world
            min_world_reached='false'
            echo min_world_reached $min_world_reached
            break
         
         # DEFLAKE RUNS
         else
            if [[ $retn_value =~ "50" && $result == "DIFFERENT"  ]]; then
               echo 'Run had different failure to the original failure'
               curr_diff_failures=$(expr $curr_diff_failures + 1)
               echo "curr_diff_failures is" $curr_diff_failures
            else
               echo; echo "Had a successful run"
               curr_successes=$(expr $curr_successes + 1)
               echo "curr_successes is" $curr_successes
            fi            
            # EXECUTE FLAKINESS RERUNS
            echo "Rerunning same world to make sure it's not flaky..."
            END=3
            flaky_successes=0
            result=""
            for i in $(seq 1 $END); do
               echo; echo "Starting rerun number" $i
               . $script_dirname/execute_run.sh $world $goalfilename $timeout $partitioning
               retn_value=$?
               cp $posefile $recent_failure
               flaky_test_reruns=$(expr $flaky_test_reruns + 1)
               total_run_count=$(expr $total_run_count + 1)
               echo "goal_epsilon is" $goal_epsilon
               
               result=$(python3 $script_dirname/compare_failures.py $orig_failure $recent_failure -e $goal_epsilon 2>&1)
               echo "retn_value is" $retn_value
               echo "flaky_test_reruns is" $flaky_test_reruns
               echo "result is" $result
               if [[ $retn_value =~ "50" ]]; then
                  echo "retn_value was " $retn_value " and result was " $result
                  if [[ $result == 'SAME' ]]; then
                     same_failures=$(expr $same_failures + 1)
                     echo "same failure; breaking..."
                     break
                  elif [[ $result == 'DIFFERENT' ]]; then
                     #curr_diff_failures=$(expr $curr_diff_failures + 1)
                     echo "different failure"
                  fi
               else
                  flaky_successes=$(expr $flaky_successes + 1)
                  #curr_successes=$(expr $curr_successes + 1)
                  echo "flaky success"
               fi
            done
            # EXAMINE FLAKINESS RERUNS
            echo; echo "Examining flakiness reruns; result is " $result
            if [[ $result == 'SAME' ]]; then
               # found similar failure --> repartition
               echo; echo "Found similar failure after flakiness reruns; repartitioning..."
               cp $world $target_world_dir/last_iter.world
               world_name=$target_world_dir/last_iter.world
               break
            else
               echo 'Flakiness reruns had different failure to the original failure --> run another world'
               curr_diff_failures=$(expr $curr_diff_failures + 1)
               echo "curr_diff_failures is" $curr_diff_failures
               # check for breaking condition
               echo; echo "Checking if this is the last of the newly generated worlds..."
               echo "world is" $world
               echo "array is" $array
               echo "len of array is" ${#array[@]}
               array=($(ls -d $target_world_dir/* | sort -V))
               index=0
               for w in "${!array[@]}"; do
                  echo $w ": ${array[$w]}"
                  if [[ "${array[$w]}" = "${world}" ]]; then
                      echo "world matches ${w}";
                      index=$w
                      break
                  fi
               done
               echo "w is" $w "and array[@]} - 2 is" $(expr ${#array[@]} - 2)
               if [[ $w =~ $(expr ${#array[@]} - 2) ]]; then
                  echo $world "is last of the newly generated worlds; repartitioning with different clusters..."
                  world_name=$target_world_dir/last_iter.world
                  break
               #else
               #   world_name=$world
               fi
            fi
         fi
      fi
      sleep 5
   done
   
   # ADDING SUCCESSES AND FAILURES TO OVERALL TALLY (FOR METRICS)
   total_successes=$(expr $successes + $curr_successes)
   total_diff_failures=$(expr $total_diff_failures + $curr_diff_failures)
   
   echo; echo 'Checking stopping conditions...'
   echo 'curr_generation_count is' $curr_generation_count
   echo 'curr_successes is' $curr_successes
   echo 'curr_diff_failures is' $curr_diff_failures
   echo 'models is' $models
   echo 'clusters is' $clusters
   echo 'world_name is' $world_name
   models=$(python3 $script_dirname/model_counter.py $world_name 2>&1)
   echo 'models is now' $models
   echo "$(expr $curr_diff_failures + $curr_successes) is" $(expr $curr_diff_failures + $curr_successes)
   curr_total_nonsim=$(expr $curr_diff_failures + $curr_successes)
   if [[ $models -ge $clusters ]]; then
      echo "models -ge clusters"
   fi
   if [[ $curr_successes -ge $curr_generation_count ]]; then
      echo "curr_successes -ge curr_generation_count"
   fi
   if [[ $curr_diff_failures -ge $curr_generation_count ]]; then
      echo "curr_diff_failures -ge curr_generation_count"
   fi
   if [[ $curr_total_nonsim -ge $curr_generation_count ]]; then
      echo "curr_total_nonsim -ge curr_generation_count"
   fi
   if [[ $curr_successes -ge $curr_generation_count && $curr_successes != 0 ]]; then
      echo "Had only successes with this partitioning"
      #world_name=$target_world_dir/last_iter.world
      #echo "Minimal world is $target_world_dir/last_iter.world"
      #cp $target_world_dir/last_iter.world $target_world_dir/minimal_world.world
      min_world_reached='true'
   elif [[ $curr_diff_failures -ge $curr_generation_count && $curr_diff_failures != 0 ]]; then
      if [ $models -le $clusters ]; then
         echo "Had only dissimilar failures with this partitioning"
         #world_name=$target_world_dir/last_iter.world
         #echo "Minimal world is $target_world_dir/last_iter.world"
         #cp $target_world_dir/last_iter.world $target_world_dir/minimal_world.world
         min_world_reached='true'
      fi
   elif [[ $curr_total_nonsim -ge $curr_generation_count ]]; then
      if [ $models -le $clusters ]; then
         echo "Had only dissimilar failures and successes with this partitioning"
         #world_name=$target_world_dir/last_iter.world
         #echo "Minimal world is $target_world_dir/last_iter.world"
         #cp $target_world_dir/last_iter.world $target_world_dir/minimal_world.world
         min_world_reached='true'
      fi
   elif [[ $curr_generation_count == 0 ]]; then
       echo "No new worlds were generated; ending..."
       return
   fi
   
   echo;
   echo "world_name is " $world_name
   clusters=$(expr $clusters + 1)
   models=$(python3 $script_dirname/model_counter.py $world_name 2>&1)
   echo "clusters:" $clusters "Models:" $models
   if [[ $clusters -gt $models ]]; then
      clusters=$models
      echo "modified number of clusters not to exceed number of models"
   fi
   
   echo "New number of clusters:" $clusters "Models:" $models
   echo "min_world_reached is" $min_world_reached
done

# Report testing metrics
echo; echo "TEST METRICS:"
#orig_models=$(python3 $script_dirname/model_counter.py $orig_world 2>$1)
#models=$(python3 $script_dirname/model_counter.py $target_world_dir/last_iter.world 2>$1)
echo 'Starting env size: ' $orig_models
python3 $script_dirname/model_counter.py $orig_world
echo 'Minimal environment size: ' $models
python3 $script_dirname/model_counter.py $target_world_dir/last_iter.world
echo "Iterations to find minimal world: " $iteration_count
echo 'Total number of worlds generated: ' $total_generation_count
echo 'Total number of tests run: ' $total_run_count
echo 'Total number of reruns due to flakiness: ' $flaky_test_reruns
#echo 'Total number of early test cutoffs: '
echo 'Total number of heterogeneous failures: ' $total_diff_failures
echo 'Total number of successful runs: ' $total_successes

cd $script_dirname



### TODO:
# python3 $script_dirname/collect_run_info.py $world_name $goalfilename $timeout $posefile $script_dirname/temp.txt $recent_failure

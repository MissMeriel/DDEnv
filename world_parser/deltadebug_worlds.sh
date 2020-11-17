#!/bin/bash

# param setup
# feed in: navigation launch file(s), timeout
eval world_name=$(pwd)/$1
echo world name is $world_name
eval goalfilename=$(pwd)/$2
combinatorial=$3
timeout=45
homedir="~"; eval homedir=$homedir
roslogfile="$homedir/.ros/log/latest/rosout.log"
msg_dirname="$homedir/husky_ws/husky_predef_msgs"
script_dirname="$homedir/husky_ws/world_parser"
goal_phrase="GOAL REACHED"
min_world_reached="false"
posefile="$homedir/husky_ws/src/husky/husky_navigation/scripts/poses.log"
cd ~/husky_ws/
alt_world_name=""
target_world_dir="$homedir/husky_ws/src/husky/husky_gazebo/worlds/subworlds"
rm $target_world_dir/*
cp $world_name $target_world_dir
cp $world_name $target_world_dir/last_iter.world
subworld_count=2
generation_count=0
iteration_count=0

while [[ ${min_world_reached} != "true" ]]; do
   
   subworld_count=$(ls $target_world_dir | wc -l)
   subworld_count=$(expr $subworld_count - 2)
   iteration_count=$(expr $iteration_count + 1)
   
   echo
   echo LOOP NUMBER $iteration_count
   
   echo subworld count is $subworld_count
   successful_run="false"
   source devel/setup.bash
   echo world name is $world_name
   
   retn_value=$(. ./execute_run.sh $world_name $goalfilename $timeout)
   #. ./world_parser/execute_run.sh $world_name $goalfilename $timeout
   retn_value=$?
   
   #sleep $timeout
   if [[ $retn_value == 40 ]]; then
      successful_run="true"
   else
      successful_run="false"
   fi
   
   # determine which subworld index we are working with
   if [[ "$world_name" =~ .*-[0-9]*.world ]]; then
      # put all worlds into array and find index
      array=($(ls $target_world_dir))
      for i in "${!array[@]}"; do
         if [[ "${array[$i]}" = "${world_name}" ]]; then
            subworld_index=$(expr ${i} + 1); break;
         fi
      done
      echo subworld_index of $world_name is $subworld_index
   else
      subworld_index=1
      echo subworld_index of original world $world_name is $subworld_index
   fi
   
   # set up next iteration
   # TODO: more specific regex for world_name?
   if [[ $successful_run == "true" &&  "$world_name" =~ .*-[0-9]*.world  ]]; then
      # check if we have tried all of the subworld files
      if [[ $subworld_index == $subworld_count ]]; then
         # all new world files produced successes; we have arrived at the minimal example (stopping condition)
         echo Minimal test environment found $homedir/husky_ws/src/husky/husky_gazebo/worlds/subworlds/last_iter.world
         cp $world_name $target_world_dir/minimal_world.world
         break
      else
         subworld_index=$(expr $subworld_index + 1)
         world_name=$target_world_dir/$(ls -t $target_world_dir | tail -n +$subworld_index | head -1)
         echo
         echo INSIDE FIRST IF
         echo This subworld hosted a successful run
         echo Lets try running the next subworld
         echo
         echo 'Choosing from: '
         ls -t $target_world_dir
         echo New world name is $world_name
         echo subworld_index is incremented to $subworld_index
      fi

   # make sure none of the other subworlds also produce a failure
   # this was the first world, not a subworld
   # NOTE: this is here for testing purposes
   # change for failure later
   elif [[ $successful_run == "false" &&  "$world_name" =~ .*-[0-9]*.world  && $subworld_count != $subworld_index ]]; then
      # rerun it to make sure it's not flaky and is actually a failure
      for i in `seq 1 3`; do
         retn_value=$(execute_run.sh $world_name $goalfilename )
         if [[ $retn_value == 40 ]]; then
            successful_run="true"
            break
         else
            successful_run="false"
         fi
      done
      # handle the flaky test
      if [[ $successful_run == "true" ]]; then
         subworld_index=$(expr $subworld_index + 1)
         world_name=$target_world_dir/$(ls -t $target_world_dir | tail -n +$subworld_index | head -1)
      else
         cp $world_name $target_world_dir/last_iter.world
         rm $target_world_dir/*2020-*.world
         if [[ $combinatorial == 'combinatorial' ]]; then
            # combinatoric baseline
            python3 $script_dirname/world_xml_parser.py $target_world_dir/last_iter.world --target_dir ${target_world_dir} -c
         else
            # trajectory aware
            python3 $script_dirname/world_xml_parser.py $target_world_dir/last_iter.world --target_dir ${target_world_dir} --trajectory $homedir/husky_ws/src/husky/husky_navigation/scripts/poses.log
         fi
         subworld_count=$(ls $target_world_dir | wc -l)
         subworld_count=$(expr $subworld_count - 2)
         subworld_index=1
         generation_count=$(expr $generation_count + $subworld_count)
         echo subworld_index is $subworld_index
         echo subworld_count is $subworld_count
         
         world_name=$target_world_dir/$(ls -t $target_world_dir | tail -n +$subworld_index | head -1)
         echo
         echo INSIDE ELIF
         echo This world produced a failure
         echo Lets try minimizing it again by half
         echo
         echo 'Choosing from: '
         ls -t $target_world_dir
         echo New world name is $world_name
         echo
      fi
   elif [[ $successful_run == "false" &&  "$world_name" =~ .*-[0-9]*_complement.world  && $subworld_count != $subworld_index ]]; then
      echo complement just run so move on to next noncomplement world
      # rerun it to make sure it's not flaky and is actually a failure
      for i in `seq 1 3`; do
         retn_value=$(execute_run.sh $world_name $goalfilename )
         if [[ $retn_value == 40 ]]; then
            successful_run="true"
            break
         else
            successful_run="false"
         fi
      done
      # handle the flaky test
      if [[ $successful_run == "true" ]]; then
         subworld_index=$(expr $subworld_index + 1)
         world_name=$target_world_dir/$(ls -t $target_world_dir | tail -n +$subworld_index | head -1)
      else
         #might need to revise this...
                  cp $world_name $target_world_dir/last_iter.world
         rm $target_world_dir/*2020-*.world
         if [[ $combinatorial == 'combinatorial' ]]; then
            # combinatoric baseline
            python3 $script_dirname/world_xml_parser.py $target_world_dir/last_iter.world --target_dir ${target_world_dir} -c
         else
            # trajectory aware
            python3 $script_dirname/world_xml_parser.py $target_world_dir/last_iter.world --target_dir ${target_world_dir} --trajectory $homedir/husky_ws/src/husky/husky_navigation/scripts/poses.log
         fi
         subworld_count=$(ls $target_world_dir | wc -l)
         subworld_count=$(expr $subworld_count - 2)
         subworld_index=1
         generation_count=$(expr $generation_count + $subworld_count)
         echo subworld_index is $subworld_index
         echo subworld_count is $subworld_count
         
         world_name=$target_world_dir/$(ls -t $target_world_dir | tail -n +$subworld_index | head -1)
         echo
         echo INSIDE ELIF
         echo This world produced a failure
         echo Lets try minimizing it again by half
         echo
         echo 'Choosing from: '
         ls -t $target_world_dir
         echo New world name is $world_name
         echo
      fi
   elif [[ $subworld_count == $subworld_index ]]; then
      # out of subworlds
      # make sure w didn't get all successful runs
      echo make sure w didnt get all successful runs
      # take the runs that were unsuccessful and spawn new threads
   else
      # We were just working with the original world
      if [[ $combinatorial == 'combinatorial' ]]; then
         # combinatoric baseline
         python3 $script_dirname/world_xml_parser.py ${world_name} --target_dir ${target_world_dir} -c
      else
         # trajectory aware
         python3 $script_dirname/world_xml_parser.py $target_world_dir/last_iter.world --target_dir ${target_world_dir} --trajectory $homedir/husky_ws/src/husky/husky_navigation/scripts/poses.log
      fi
      world_name=$target_world_dir/$(ls -t $target_world_dir | tail -n +1 | head -1)
      subworld_count=$(ls $target_world_dir | wc -l)
      subworld_count=$(expr $subworld_count - 2)
      generation_count=$(expr $generation_count + $subworld_count)
      echo
      echo INSIDE ELSE
      echo This world hosted a successful run
      echo Lets try running on a subworld
      echo New world name is $world_name
      echo
   #elif [[ successful_run == "false" and end position too close to start ]]; then
      # seems like robot just oscillated in place
      # probably a flaky test
      # run it two more times with this world file
   # failure on a subworld
   # make new world files and repeat
   fi
   
   # make sure you're not running tests on empty world
   subworld_count=$(ls $target_world_dir | wc -l)
   subworld_count=$(expr $subworld_count - 2)
   if [[ $subworld_count == "0" ]]; then
      echo
      echo World is empty
      echo Subworld generation terminating...
      break
   fi
   
   #sleep 3
done
echo "Iterations to find minimal world: " $iteration_count
echo 'Total number of worlds generated: ' $generation_count
python3 world_parser/model_counter.py $target_world_dir/minimal_world.world
cd ~/husky_ws/world_parser
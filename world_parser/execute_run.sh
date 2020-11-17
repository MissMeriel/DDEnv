#!/bin/bash
### N.B. must have shutter installed
### sudo apt install shutter
###  sudo apt install libnet-dbus-glib-perl


world_name=$1
goalfilename=$2
timeout=$3
partition=$4
#ordering=$5
laser_enabled=$6
successful_run="false"
early_cutoff="false"
homedir="~"; eval homedir=$homedir
posefile="$homedir/husky_ws/src/husky/husky_navigation/scripts/poses.log"
rm $posefile
goal_phrase="GOAL REACHED"
scan_topic="/remapped_scan"
if [[ "$orig_world" == *"friction"* ]]; then
   scan_topic="/scan"
   x="-0.1"
   y="-0.3"
   z="10.0"
   echo "scan topic is " $scan_topic
else
   x="-3.0"
   y="-0.5"
   z="10.0"
   #x="-5.0"
   #y="-3.0"
   #z="10.0"
   #scan_topic="/scan"
fi

rosbag record -a -o ./presentation_rubble.bag  __name:=my_bag &

#TODO: further parameterize these commands
#echo "laser_enabled is " $laser_enabled
roslaunch husky_gazebo husky_rough_terrain.launch world_name:=$world_name x:=$x y:=$y z:=$z scan_topic:=$scan_topic &
#roslaunch husky_gazebo husky_rough_terrain.launch world_name:=$world_name laser_enabled:=$laser_enabled &
sleep 15

roslaunch husky_navigation amcl_rough_terrain_fakescan.launch &
#roslaunch husky_navigation gmapping_demo.launch &
#rosrun husky_navigation log_for_dd.py &
sleep 7
echo "publishing message from" $goalfilename
rostopic pub -s /move_base_simple/goal -r 50  geometry_msgs/PoseStamped -f $goalfilename &

#echo; echo "goal phrase is " $goal_phrase
echo
# log watching while run executes
#sleep 45 == sleep 3 * 15
for i in `seq 1 15`; do
   sleep 3
   if grep -q "$goal_phrase" "$posefile"; then
      echo "Goal phrase found in $posefile"
      echo "Early cutoff"
      successful_run="true"
      early_cutoff="true"
      break
   else
      echo "No goal phrase found in $posefile"
   fi
done 

# take screenshot
outfile="${world_name/.world/$partitioning_$ordering}"
outfile="${outfile/subworlds/images}" 
shutter -f -o $outfile.png -e
#sleep 3

rosnode kill /my_bag

echo
echo Killing processes....
# kill gazebo processes
killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient
# kill ros processes
killall -9 amcl & killall -9 roslaunch & killall -9 map_server & killall -9 move_base  & killall -9 rosout
killall -9  twist_marker_server & killall -9  robot_state_publisher & killall -9  twist_mux
kill $(ps aux | grep rosmaster | awk '{print $2}')
kill $(ps aux | grep husky_joint_publisher | awk '{print $2}')
kill $(ps aux | grep ekf_localization_node | awk '{print $2}')
kill $(ps aux | grep twist_marker_server | awk '{print $2}')
kill $(ps aux | grep fake_scan | awk '{print $2}')
kill $(ps aux | grep pose_logger | awk '{print $2}')
kill $(ps aux | grep spawn_model | awk '{print $2}')
kill $(ps aux | grep husky | awk '{print $2}')
kill $(ps aux | grep rosbag | awk '{print $2}')
sleep 3

python3 $script_dirname/compare_failures_test.py $orig_failure $posefile -e $goal_epsilon

if [[ $successful_run =~ "true" && $early_cutoff =~ "true" ]]; then
   # change to provide this metric
   return 40
elif [[ $successful_run =~ "true" ]]; then
   return 40
else
   return 50
fi
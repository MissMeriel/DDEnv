#!/bin/bash

### SCENARIOS:
# rubble
# rough terrain (static)
# rough terrain (dynamic)
# ditch
# clearpath playpen
# frictionpatch

### FAILURES:
# DITCH: $script_dirname/failures/rough_terrain_static_origfailure_correct1.log
# RUBBLE:
# FRICTION: 

cd ~/husky_ws/world_parser
eval results_dir=~/husky_ws/world_parser/results
eval world_dir=~/husky_ws/src/husky/husky_gazebo/worlds
echo "Results will appear in " $results_dir


### TABLE #1: MODEL2MODEL + CRASH
#echo "Running model2model clustering and crash_proximal ordering with rough_terrain_gap.world... (started `date`.)"
#{ time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/rough_terrain_gap.world ../husky_predef_msgs/goal_ditch model2model crash_proximal 0.2 &> $results_dir/roughterrainstatic_model2model_crashproximal.log; } &>> $results_dir/roughterrainstatic_model2model_crashproximal.log
##cp $world_dir/last_iter.world $results_dir/roughterrainstatic_model2model_crashproximal.world
#echo "Running model2model clustering and crash_proximal ordering with rubble_loose11.world... (started `date`.)"
#{ time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/rubble_loose11.world ../husky_predef_msgs/goal_ditch model2model crash_proximal 0.3 &> $results_dir/rubble_model2model_crashproximal.log; }  &>> $results_dir/rubble_model2model_crashproximal.log
##cp $world_dir/last_iter.world $results_dir/rubble_model2model_crashproximal.world
#echo "Running model2model clustering and crash_proximal ordering with frictionpatch.world... (started `date`.)"
#echo "(This took over 18hrs to run last time)"
#{ time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/frictionpatch6.world ../husky_predef_msgs/goal_friction model2model crash_proximal 0.33  &> $results_dir/friction_model2model_crashproximal.log ; } &>> $results_dir/friction_model2model_crashproximal.log
##cp $world_dir/last_iter.world $results_dir/friction_model2model_crashproximal.world


### TABLE #2: MODEL2MODEL + AVGTRAJECTORY
echo "Running model2model clustering and avg_trajectory_distance ordering with rough_terrain_gap.world... (started `date`.)"
{ time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/rough_terrain_gap.world ./husky_predef_msgs/goal_ditch model2model avg_trajectory 0.75  &> $results_dir/roughterrainstatic_model2model_avgtrajectory.log ; } &>> $results_dir/roughterrainstatic_model2model_avgtrajectory.log
##cp $world_dir/last_iter.world $results_dir/roughterrainstatic_model2model_avgtrajectory.world
#echo "Running model2model clustering and avg_trajectory_distance ordering with frictionpatch.world... (started `date`.)"
#{ time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/frictionpatch6.world ../husky_predef_msgs/goal_friction model2model crash_proximal 0.33  &> $results_dir/friction_model2model_avgtrajectory.log ; } &>> $results_dir/friction_model2model_avgtrajectory.log
##cp $world_dir/last_iter.world $results_dir/friction_model2model_avgtrajectory.world
###time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/ditch.world ../husky_predef_msgs/goal_ditch model2model avg_trajectory 0.33 &> $results_dir/ditch_model2model_avgtrajectory.log


### TABLE #3.1: TRAJECTORY SEG + CRASH
#echo "Running trajectory_segment clustering and crash_proximal ordering with rough_terrain_gap.world... (started `date`.)"
#{ time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/rough_terrain_gap.world ../husky_predef_msgs/goal_ditch trajectory_seg crash_proximal 0.33 &> $results_dir/roughterrainstatic_trajectoryseg_crashproximal.log ; } &>> $results_dir/roughterrainstatic_trajectoryseg_crashproximal.log
##cp $world_dir/last_iter.world $results_dir/roughterrainstatic_trajectoryseg_crashproximal.world
#echo "Running trajectory_seg clustering and crash_proximal ordering with rubble_loose11.world... (started `date`.)"
#{ time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/rubble_loose11.world ../husky_predef_msgs/goal_ditch trajectory_seg crash_proximal 0.33 &> $results_dir/rubble_trajectoryseg_crashproximal.log ; } &>> $results_dir/rubble_trajectoryseg_crashproximal.log
##cp $world_dir/last_iter.world $results_dir/rubble_trajectoryseg_crashproximal.world
#echo "Running trajectory_segment clustering and crash_proximal ordering frictionpatch.world..."
#{ time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/frictionpatch6.world ../husky_predef_msgs/goal_friction trajectory_seg crash_proximal 0.33  &> $results_dir/friction_trajectoryseg_crashproximal.log ; } &>> $results_dir/friction_trajectoryseg_crashproximal.log


### TABLE #3.2: TRAJECTORY SEG + AVGTRAJECTORY
#echo "TABLE 3: TRAJECTORY SEG + AVGTRAJECTORY"
#echo "Running trajectory_segment clustering and crash_proximal ordering with rough_terrain_gap.world... (started `date`.)"
#{ time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/rough_terrain_gap.world ./husky_predef_msgs/goal_rough_terrain2 trajectory_seg avg_trajectory 0.2 &> $results_dir/roughterrainstatic_trajectoryseg_avgtrajectory2.log ; } &>> $results_dir/roughterrainstatic_trajectoryseg_avgtrajectory2.log
#cp $world_dir/last_iter.world $results_dir/roughterrainstatic_trajectoryseg_crashproximal.world
#echo "Running trajectory_segment clustering and crash_proximal ordering frictionpatch.world..."
#{ time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/frictionpatch6.world ../husky_predef_msgs/goal_friction trajectory_seg avg_trajectory 0.33  &> $results_dir/friction_trajectoryseg__avgtrajectory.log ; } &>> $results_dir/friction_trajectoryseg_crashproximal.log

#echo "Running trajectory_seg clustering and crash_proximal ordering with rubble_loose11.world... (started `date`.)"
#{ time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/rubble_loose11.world ../husky_predef_msgs/goal_ditch trajectory_seg avg_trajectory 0.33 &> $results_dir/rubble_trajectoryseg_avgtrajectory.log ; } &>> $results_dir/rubble_trajectoryseg_crashproximal.log
#cp $world_dir/last_iter.world $results_dir/rubble_trajectoryseg_crashproximal.world


# TABLE 0: COMBINATORIAL + RANDOM
#time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/rubble_loose11.world ../husky_predef_msgs/goal_rubble_loose2 combinatorial random 0.2 &> $results_dir/rubble_combinatorial.log
#time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/rough_terrain_gap.world ../husky_predef_msgs/goal_ditch combinatorial random 0.2 &> $results_dir/roughterrainstatic_combinatorial.log
#time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/rough_terrain_gap_dynamic_min3.world ../husky_predef_msgs/goal_ditch combinatorial random 0.2  &> $results_dir/roughterraindynamic_combinatorial.log
##time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/ditch.world ../husky_predef_msgs/goal_rough_terrain2 combinatorial random 0.2 &> $results_dir/ditch_combinatorial.log
##time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/clearpath_playpen.world ../husky_predef_msgs/goal_playpenorig combinatorial random 0.2 &> > $results_dir/playpen_combinatorial.log
#time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/frictionpatch.world ../husky_predef_msgs/goal_friction combinatorial random  0.2 &> $results_dir/friction_combinatorial.log


### TABLE #4: MODEL2MODEL + MINTRAJECTORY
#time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/rubble_loose4.world ../husky_predef_msgs/goal_rubble_loose2 model2model min_trajectory &> > $results_dir/rubble_model2model_mintrajectory.log
#time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/rough_terrain_gap.world ../husky_predef_msgs/goal_ditch model2model min_trajectory &> > $results_dir/roughterrainstatic_model2model_mintrajectory.log
#time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/rough_terrain_gap_dynamic_min3.world ../husky_predef_msgs/goal_ditch model2model min_trajectory &> > $results_dir/roughterraindynamic_model2model_mintrajectory.log
#time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/ditch.world ../husky_predef_msgs/goal_ditch model2model min_trajectory &> > $results_dir/ditch_model2model_mintrajectory.log
#time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/frictionpatch.world ../husky_predef_msgs/goal_friction model2model min_trajectory  &> > $results_dir/friction_model2model_mintrajectory.log


# trajectory_segment clustering
# crash_proximal ordering
#echo "Running trajectory_segment clustering and crash_proximal ordering with rough_terrain_gap_dynamic_min3.world..."
#{ time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/rough_terrain_gap_dynamic_min3.world ../husky_predef_msgs/goal_ditch trajectory_seg crash_proximal 0.2 &> $results_dir/roughterraindynamic_trajectoryseg_crashproximal.log ; } &>> $results_dir/roughterraindynamic_trajectoryseg_crashproximal.log
##time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/ditch.world ../husky_predef_msgs/goal_ditch trajectory_seg crash_proximal &> > $results_dir/ditch_trajectoryseg_crashproximal.log
##time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/clearpath_playpen.world ../husky_predef_msgs/goal_playpen trajectory_seg crash_proximal &> > $results_dir/playpen_trajectoryseg_crashproximal.log


### ALL STATIC DITCH SCENARIOS


### ALL DYNAMIC DITCH SCENARIOS
#echo "Running model2model clustering and crash_proximal ordering with rough_terrain_gap_dynamic_min3.world..."
#{ time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/rough_terrain_gap_dynamic_min3.world ../husky_predef_msgs/goal_ditch model2model crash_proximal 0.2 &> $results_dir/roughterraindynamic_model2model_crashproximal.log; } &>> $results_dir/roughterraindynamic_model2model_crashproximal.log
#echo "Running model2model clustering and avg_trajectory_distance ordering with rough_terrain_gap_dynamic_min3.world..."
#{ time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/rough_terrain_gap_dynamic_min3.world ../husky_predef_msgs/goal_ditch model2model avg_trajectory 0.2 &> $results_dir/roughterraindynamic_model2model_avgtrajectory.log ; } &>> $results_dir/roughterraindynamic_model2model_avgtrajectory.log


### ALL MINI DITCH SCENARIOS

### ALL CLEARPATCH PLAYPEN SCENARIOS
#time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/clearpath_playpen.world ../husky_predef_msgs/goal_playpenorig model2model avg_trajectory &> > $results_dir/playpen_model2model_avgtrajectory.log
#time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/clearpath_playpen.world ../husky_predef_msgs/goal_playpenorig model2model min_trajectory &> > $results_dir/playpen_model2model_mintrajectory.log


### ALL RUBBLE SCENARIOS
#echo "Running model2model clustering and crash_proximal ordering with rubble_loose11.world... (started `date`.)"
#{ time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/rubble_loose11.world ../husky_predef_msgs/goal_ditch model2model crash_proximal 0.3 &> $results_dir/rubble_model2model_crashproximal.log; }  &>> $results_dir/rubble_model2model_crashproximal.log
#cp $world_dir/last_iter.world $results_dir/rubble_model2model_crashproximal.world
#echo "Running model2model clustering and avg_trajectory ordering with rubble_loose11.world..."
#{ time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/rubble_loose11.world ../husky_predef_msgs/goal_ditch model2model avg_trajectory 0.3 &> $results_dir/rubble_model2model_avgtrajectory.log ; } &>> $results_dir/rubble_model2model_avgtrajectory.log
#echo "Running trajectory_seg clustering and crash_proximal ordering with rubble_loose11.world... (started `date`.)"
#{ time . ./dd_dfs2.sh ../src/husky/husky_gazebo/worlds/rubble_loose11.world ../husky_predef_msgs/goal_ditch trajectory_seg crash_proximal 0.3 &> $results_dir/rubble_trajectoryseg_crashproximal.log ; } &>> $results_dir/rubble_trajectoryseg_crashproximal.log
#cp $world_dir/last_iter.world $results_dir/rubble_trajectoryseg_crashproximal.world

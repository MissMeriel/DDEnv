# gazebo_world_deltadebug

## Worlds, spawn poses, and goal messages
All worlds can be found in 'src/husky/husky_gazebo/worlds/'. These commands were meant to be run from the 'husky_ws/world_parser' directory.

1. Clearpath playpen
'''
. ./deltadebug_worlds.sh ../src/husky/husky_gazebo/worlds/clearpath_playpen.world ../husky_predef_msgs/goal_playpen [combinatorial]
'''

2. Ditch scenario
'''
. ./deltadebug_worlds.sh ../src/husky/husky_gazebo/worlds/ditch.world ../husky_predef_msgs/goal_rough_terrain_stuck
'''
3. Rubble scenario
'''
. ./deltadebug_worlds.sh ../src/husky/husky_gazebo/worlds/rubble_loose3.world ../husky_predef_msgs/goal_rubble_loose3
'''

4. Friction scenario
'''
. ./deltadebug_worlds.sh ../src/husky/husky_gazebo/worlds/friction.world ../husky_predef_msgs/goal_friction
'''

## Bonus worlds, spawn poses, and goal messages

1. Mountain scenario
'''
. ./deltadebug_worlds.sh ../src/husky/husky_gazebo/worlds/valley.world ../husky_predef_msgs/goal1
'''
<!--
2. Rough terrain + ditch scenario
'''
. ./deltadebug_worlds.sh ../src/husky/husky_gazebo/worlds/
'''


3. Broken laser scanner
'''
. ./deltadebug_worlds.sh ../src/husky/husky_gazebo/worlds/
'''

4. Robot tipover
'''
. ./deltadebug_worlds.sh ../src/husky/husky_gazebo/worlds/
'''
-->



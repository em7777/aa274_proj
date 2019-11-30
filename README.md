# aa274_proj

# Installation
1. Clone to a new catkin_ws/src
2. cd catkin_ws && catkin_make

# Usage 
1.  Launch the tb stack, gmapping, navigation etc.


`roslaunch asl_turtlebot project_custom.launch`

2.  Start the state machine


`roscd executive_smach/scripts`


`python aa274_sm.py`

3. Start the state machine viewer


`rosrun smach_viewer smach_viewer.py`


## individual modules:
Frontier Exploration 
Will explore map until complete. Launches the tb stack, gmapping, navigation, frontier exploration etc. and assigns the exploration boundary in rviz automatically.
`roslaunch asl_turtlebot project_custom.launch`

Known Space Exploration
This will explore map until fully re-explored only once. Use after the entire map has been explored and if we have not identified all food vendors yet.


`roslaunch asl_turtlebot frontier_expl.launch known_map:=true`

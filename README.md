# aa274_proj

# Installation
1. Clone to a new catkin_ws/src
2. cd catkin_ws && catkin_make

# Frontier Exploration (Will explore map until complete)
`roslaunch asl_turtlebot project_custom.launch`

## Known Space Exploration
This will explore map until fully re-explored only once. Use after the entire map has been explored and if we have not identified all food vendors yet.


`roslaunch asl_turtlebot frontier_expl.launch known_map:=true`

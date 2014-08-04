bool position             # true for a position command, false for a velocity command
bool armCommand           # true if this command includes arm joint inputs
bool fingerCommand        # true if this command includes finger inputs
bool repeat               # true if the command should be repeatedly sent over a short interval
geometry_msgs/Twist arm   # position (m, rad) or velocity (m/s, rad/s) arm command
float32[] fingers         # position (rad) or velocity (rad/s) finger command

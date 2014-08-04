bool position       # true for a position command, false for a velocity command
bool armCommand     # true if this command includes arm joint inputs
bool fingerCommand  # true if this command includes finger inputs
bool repeat         # true if the command should be repeatedly sent over a short interval
float32[] joints    # position (rad) or velocity (rad/s) arm command
float32[] fingers   # position (rad) or velocity (rad/s) finger command


startQ = [pi/2 pi/4 pi/2 pi/4 pi/2];
goalQ = [pi/8 3*pi/4 pi 0.9*pi 1.5*pi];
planner_id = 0 % placeholder for now
runtest('map1.txt',startQ, goalQ, planner_id);
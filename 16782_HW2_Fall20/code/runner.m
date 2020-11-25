startQ = [pi/1.75 pi/6 pi/1.5 pi/1.65 pi/1.8];
goalQ =  [pi/8 3*pi/4 pi 0.9*pi 1.5*pi];
planner_id = 2 % placeholder for now
%for iter=1:10
runtest('map2.txt',startQ, goalQ, planner_id);
%end

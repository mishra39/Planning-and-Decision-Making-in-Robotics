best_start = [pi/2 pi/4 pi/2 pi/4 pi/2];
best_goal = [pi/2 pi/4 pi/2 pi/4 pi/2];
start = [pi/1.8 pi/6 pi/1.7 pi/1.5 pi/2;
	  pi/1.7 pi/6 pi/1.5 pi/1.5 pi/2;
	  pi/1.75 pi/6 pi/1.5 pi/1.5 pi/2;
	  pi/1.7 pi/6 pi/1.5 pi/1.65 pi/1.8;
	  pi/1.7 pi/6 pi/1.5 pi/1.5 pi/1.8];

goal = [pi/8 3*pi/4 pi 0.9*pi 1.5*pi;
	pi/7.8 3*pi/4 pi 0.9*pi 1.5*pi;
	pi/8 3*pi/4 pi 0.9*pi 1.5*pi;
	pi/8 3*pi/4 pi 0.9*pi 1.25*pi;
	pi/8 3*pi/4 pi 0.9*pi 1.5*pi];
planner_id = 1; 
i = 2;
runtest('map2.txt',start(i,:), goal(i,:), planner_id);
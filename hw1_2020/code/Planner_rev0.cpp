/*

double start = 0;
bool pathSearch::isValid(int x, int y)
{
    return (x >0) && (x<=x_size) && (y>0) && (y <= y_size);
}

bool pathSearch::isFree(int newx, int newy)
{
    if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))
    {
        return true;
    }

    else
    {
        return false;
    }
}

double pathSearch::hueristic_func()
{
    // Ideal Heuristic
    double x_dist = (double) (fabs(robotposeX-goalposeX));
    double y_dist = (double) (fabs(robotposeY-goalposeY));
    return (max(x_dist, y_dist));
}

/* Do backwards a* with multiple start states in 2d space to determine our heuristic values, where we have a single open list that contains all the start states.
Our stop condition on our expansion loop is when nothing is left in our open list.*/
void backSearch()
{
    // Add all states from target tarjectory to the open list
    
}

void pathSearch::computePath() // function responsible for mainly implementing A star and finding the shortest pathSearch
{
    while (!open.empty()) // ???? how to check if s_goal has been expanded, i.e. if it is present in closed list
    {
        pair<double,int> currCell = open.top(); // get the f value and index of the top cell in the queue
        open.pop(); // remove the cell from the queue

        // Check to see if the cell has already been expanded
        if(!closed[currCell.second])
        {
            closed[currCell.second] = true;    // add the cell to the visited list
            pair<int,int> curr_xy = getXY(currCell.second); // get x and y coordinates of the current index

            for (int dir = 0; dir < NUMOFDIRS; dir++)
            { 
                // Loop over all 8 successors
                int newx = curr_xy.first + dX[dir];
                int newy = curr_xy.second + dY[dir];
                
                if (isValid(newx, newy) && isFree(newx, newy)) // check to ensure that the cell is collision free and inside the map
                {
                    int newIdx = getXYidx(newx, newy); // find index of new x,y coordinates
                    if (cellVal[newIdx].g_val > cellVal[currCell.second].g_val + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)]) // Check to see if g_new is greater than g_curr + cost of going from curr to new
                    {
                        cellVal[newIdx].g_val = cellVal[currCell.second].g_val + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)]; // set the lower cost as new g value
                        cellVal[newIdx].h_val = hueristic_func();
                        cellVal[newIdx].f_val = cellVal[newIdx].g_val + cellVal[newIdx].h_val;
                        open.push(make_pair(cellVal[newIdx].f_val,newIdx)); // inset this new cell into the open list to be compared with other successors
                        cellVal[newIdx].parent = currCell.second; // index of the parent for this successor
                    }
                }
            }
            
        }
    }
}

/* Returns the number of steps required to reach to robot from current goal */
int pathSearch::goalToRobotSteps()
{
    int tot_steps = 0; // counter for total steps
    int cellParent = getXYidx(goalposeX, goalposeY);
    int robotIdx = getXYidx(robotposeX, robotposeY); // position of the robot
    while (cellParent != robotIdx)
    {
        tot_steps++;
        cellParent = cellVal[cellParent].parent;
    }
    return tot_steps;
}

void pathSearch::bestGoal(auto time_start) // Function to find points in the target's trajectory to intercept  
{
    // Approach 1
    //goalposeX = (int) target_traj[target_steps-1];
    //goalposeY = (int) target_traj[target_steps-1+target_steps];
    // Approach 2
    // find the time elapsed so far by the search algorithm
    end = high_resolution_clock::now();
    auto elapsed_secs = duration_cast<seconds> (end-time_start); // time elapse so far
    int exec_durn = 0; // time that might be spent in the execution for this goal state
    int best_durn = INT_MAX; // Variable to store the fastest execution so far
    int best_goal = 0; // index of the best goal so far

    // Estimate how much the target has moved in this time
    for (int i = 0; i < target_steps; i++)
    {
        goalposeX = (int) target_traj[i]; // x coordinate
        goalposeY = (int) target_traj[i+target_steps]; // y coordinate
        exec_durn = elapsed_secs + goalToRobotSteps(); 
        if (exec_durn < i)
        {   
            if (exec_durn < best_durn)
            {
                best_goal = getXYidx(goalposeX, goalposeY);
                best_durn = exec_durn;
            }
        }
    }
    // Save the best goal position found so far
    goalposeX = getXY(best_goal).first;
    goalposeY = getXY(best_goal).second;
}

void pathSearch::GoaltoRobot() // function to back track from found goal to start point
{
    int goalIdx = getXYidx(goalposeX, goalposeY);    // find index of goal position
    int robotIdx = getXYidx(robotposeX, robotposeY); // find the index of robot position
    while (cellVal[goalIdx].parent != robotIdx)
    {
        GoaltoRobotPath.push_back(goalIdx);
        goalIdx = cellVal[goalIdx].parent;
    }
    GoaltoRobotPath.push_back(goalIdx); // push the robot index to the path as well
}

 void pathSearch::startInit() // Initialize start cell
    {
        cellVal[getXYidx(robotposeX, robotposeY)].g_val = 0; // starting cell has a g value of zero
        this->open.push(make_pair(0,getXYidx(robotposeX,robotposeY))); // Initialize with f value of zero
    }

pair<int,int> aStarFunc(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{
    pathSearch aStar(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, target_traj, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    auto start = high_resolution_clock::now();
    aStar.startInit();
    aStar.computePath();
    aStar.bestGoal(start);
    aStar.GoaltoRobot();

    // Check to see if goal reached
    if (robotposeX == aStar.goalposeX && robotposeY == aStar.goalposeY)
    {
        return make_pair(robotposeX, robotposeY); // Don't move if at the goal
    }
}

static void planner(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
   // mainFunc(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, target_traj, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // for now greedily move towards the final target position,
    // but this is where you can put your planner
    
    int goalposeX = (int) target_traj[target_steps-1];
    int goalposeY = (int) target_traj[target_steps-1+target_steps];
    // printf("robot: %d %d;\n", robotposeX, robotposeY);
    // printf("goal: %d %d;\n", goalposeX, goalposeY);

    int bestX = 0, bestY = 0; // robot will not move if greedy action leads to collision
    double olddisttotarget = (double)sqrt(((robotposeX-goalposeX)*(robotposeX-goalposeX) + (robotposeY-goalposeY)*(robotposeY-goalposeY)));
    double disttotarget;
    for(int dir = 0; dir < NUMOFDIRS; dir++)
    {
        int newx = robotposeX + dX[dir];
        int newy = robotposeY + dY[dir];

        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
        {
            if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
            {
                disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
                if(disttotarget < olddisttotarget)
                {
                    olddisttotarget = disttotarget;
                    bestX = dX[dir];
                    bestY = dY[dir];
                }
            }
        }
    }
    robotposeX = robotposeX + bestX;
    robotposeY = robotposeY + bestY;
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;
    
    return;
}
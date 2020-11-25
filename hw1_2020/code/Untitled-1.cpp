/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include<bits/stdc++.h> 
#include <queue>
#include <chrono>
#include <mex.h>
#include <unordered_map>
#include<time.h>
#include <iostream>
#include "SearchCell.hpp"

/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]


/* Output Arguments */
#define	ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))
#define GETTIMEINDEX(X, Y, T, XSIZE, YSIZE, TIMETOT) ((Y-1)*XSIZE*TIMETOT + (X-1) + (T-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

using namespace std;
using namespace std::chrono;

class greaterG{
    public:
        int operator() (const SearchCell* cl, const SearchCell* cr) // cl: left cr: right
        {
            // this acts as greater in pq so we create a min-heap
            return (cl->g_val) > (cr->g_val);
        }
};

class greaterH{
    public:
        int operator() (const SearchCell* cl, const SearchCell* cr) // cl: left cr: right
        {
            // this acts as greater in pq so we create a min-heap
            return (cl->g_val + 10*cl->h_val) > (cr->g_val + 10*cr->h_val);
        }
};

//  Find the heuristic values of all the cells
/******Heuristic Value here will be the same for the optimal g-value as we are doing a backward search*********************/
unordered_map<int,int> h_calc(double* map, int collision_thresh, double* target_traj, int x_size, int y_size,int target_steps)
{
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    int targetIdx, newIdx; // target point and new neighbor index on trajectory
    SearchCell* targetCell, *newCell; // Pointer to the cell of the point on trajectory
    priority_queue<SearchCell* , vector<SearchCell*>, greaterG > open_pq;
    unordered_map<int,bool> closed; // cell index and bool parameter to see if cell is visited already
    unordered_map<int,SearchCell*> open_map; // map to check which nodes have been visited before but have not been fully expanded yet (not found the optimal f value yet)
    unordered_map<int,int> h_map; // Map to store the heuristic values from the backward Search
    
    // Add all the trajectory points to the open list->Checked
    for (int i = 0; i < target_steps; i++)
    {
        targetIdx = GETMAPINDEX(target_traj[i], target_traj[i+target_steps], x_size, y_size);  // get the idx/key of the target point
        targetCell = new SearchCell(target_traj[i], target_traj[i+target_steps]);  // Create a Searchcell pointer using current position
        targetCell->g_val = 0; // Initiate the root/goal with zero for backward search
        targetCell->cost_val =(int) map[targetIdx]; // Cost of the current position in the map
        h_map[targetIdx] = targetCell->g_val;
/*
        targetObj.x_pos =  target_traj[i];  // Create a Searchcell pointer using current position
        targetObj.y_pos =  target_traj[i+target_steps];
        targetObj.cost_val =(int) map[targetIdx]; // Cost of the current position in the map
        targetObj.g_val = 0; // Initiate the root/goal with zero for backward search
        targetCell = &targetObj;*/
        mexPrintf("SearchCell created \n");
        h_map[targetIdx] = targetCell->g_val;
        open_pq.push(targetCell);
    }

    // Find heuristic using Dijsktra search
    while (!open_pq.empty())
    {
        // All cells have g value of zero so now each starting cell will expand all the states on the map
        // remove the cell with lowest f_value top node amd append to closed list as expanded 
        targetCell = open_pq.top(); 
        open_pq.pop();
        targetIdx = GETMAPINDEX(targetCell->x_pos, targetCell->y_pos, x_size, y_size); // Get the index of the current cell to check in closed list
    
        if (!closed[targetIdx]) // Check the closed list to see if the state has been expanded already
        {
            closed[targetIdx] = true; // mark the cell as expanded
            
            // Explore all the neighbors on the 8-connected grid
            for (int dir = 0; dir < NUMOFDIRS; dir++)
            { 
                // Loop over all 8 successors
                int newx = targetCell->x_pos + dX[dir];
                int newy = targetCell->y_pos + dY[dir];
                newIdx = GETMAPINDEX(newx,newy,x_size,y_size); // find the index of the new cell
                
                if ((newx >=1 && newx <= x_size && newy >=1 && newy <= y_size) && ((int) map[newIdx] < collision_thresh)) // check to ensure that the cell is collision free and inside the map
                {
                    if (closed[newIdx])// check to see if the neighbor has already been added to the closed list
                    {
                        continue;
                    }
                    
                    if (open_map[newIdx]) // Update the g_value if the cell has been expanded before
                    {
                        newCell = open_map[newIdx];
                        if (newCell->g_val > targetCell->g_val + newCell->cost_val) // update if g* found
                        {
                            newCell->g_val = targetCell->g_val + newCell->cost_val;
                            h_map[newIdx] = newCell->g_val; // update the heuristic value
                            open_pq.push(newCell); // add the successor to the open list to be expanded next
                        }
                    }
                                                                                                                         
                    else // if the cell is being visited for the first time
                    {
                        newCell = new SearchCell(newx,newy); // create a new cell
                        newCell->cost_val = (int) map[newIdx]; // assign the cost to the successor
                        newCell->g_val = targetCell->g_val + newCell->cost_val;
                        h_map[newIdx] = newCell->g_val;
                        open_map[newIdx] = newCell; // mark the cell as Initiated
                        open_pq.push(newCell);
                    }
                }
            }
        }
    }
    mexPrintf("Backward Dijsktra Complete \n");
    return h_map; // return the map of heuristic values
}

/* This function takes into account the total time elapsed so far and checks if the target has/will ended up at the same location in that time*/ 
bool isGoal(SearchCell* currCell, int target_steps, double* target_traj,int timeSoFar)
{
    // Location of the target after combining the steps required to reach the current cell and the elapsed time
    int targetX = target_traj[currCell->t_val + timeSoFar];
    int targetY = target_traj[currCell->t_val + timeSoFar + target_steps];

    if (currCell->x_pos == targetX && currCell->y_pos == targetY)
    {
        return true;
    }
    else
    {
        return false;
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
    // 8-connected grid and the last x and y positions to stay put at a location
    int dX[9] = {-1, -1, -1,  0,  0,  1, 1, 1, 0};
    int dY[9] = {-1,  0,  1, -1,  1, -1, 0, 1, 0};

    static unordered_map<int,pair<int,int>> action_map; // map for actions
    static pair<int,int> actionXY;
    int idx;
    static int timeCount;

    if (curr_time == 0)
    {
        priority_queue<SearchCell* , vector<SearchCell*>, greaterH> open_pq;
        unordered_map<int,bool> closed; // cell index and bool parameter to see if cell is visited already
        unordered_map<int,SearchCell*> visited; // map to check which nodes have been visited before but have not been fully expanded yet (not found the optimal f value yet)
        unordered_map<int,int> h_map; // Map to store the heuristic values from the backward Search
        int currX, currY, currIdx, adjX, adjY, adjIdx; // target point and new neighbor index on trajectory
        int elapsed_time, currCelltime; // time elapsed so far
        SearchCell *currCell, *parentCell, *adjCell,*startCell; // Pointers to the current, neighbor, and previous cells
        timeCount = 0;
        time_t begin,end; // start and end time of planning
        begin = time(nullptr);
        // Backward Dijsktra search call to find heuristic for all cells on the map->Checked
        h_map = h_calc(map, collision_thresh, target_traj, x_size, y_size, target_steps);

        startCell = new SearchCell(robotposeX,robotposeY); // initialize the starting cell with robotpose
        startCell->t_val = curr_time;
        open_pq.push(startCell);

        // Foraward A* Search
        while (!open_pq.empty())
        {   // Remove the cell with minimum h value
            currCell = open_pq.top();
            open_pq.pop();
            currIdx = GETTIMEINDEX(currCell->x_pos, currCell->y_pos, currCell->t_val, x_size,y_size, target_steps);
            if (!closed[currIdx]) // check if the cell is alread in closed list
            {
                closed[currIdx] = true; // add popped cell to the closed list
                end = time(nullptr);
                elapsed_time = end - begin + 1;
                if (isGoal(currCell, target_steps, target_traj, elapsed_time)) // check to see if the current cell is the goal
                {
                    //mexPrintf("Goal found \n");
                    break;
                }

                else
                {
                  //  mexPrintf("Goal not found \n");
                }
                

                for (int dir = 0; dir < 9; dir++)
                { 
                    // Loop over all 8 successors and the stationary positions
                    int newx = currCell->x_pos + dX[dir];
                    int newy = currCell->y_pos + dY[dir];
                    adjIdx = GETTIMEINDEX(newx,newy, currCell->t_val + 1 ,x_size, y_size, target_steps); // currCelltime + 1 is the time to reach the neighbor
                    
                    if ((newx >=1 && newx <= x_size && newy >=1 && newy <= y_size) && ((int) map[GETMAPINDEX(newx, newy, x_size, y_size)] < collision_thresh)) // check to ensure that the cell is collision free and inside the map
                    {
                        if (closed[adjIdx])
                        {
                            continue;
                        }
                     
                        if (visited[adjIdx]) // if the cell has been visited before but hasn't been popped yet, see if g value can be updated
                        {
                            adjCell = visited[adjIdx];
                            if (adjCell->g_val > currCell->g_val + adjCell->cost_val)
                            {
                                adjCell->g_val = currCell->g_val + adjCell->cost_val; // update with lower g value
                                adjCell->parent = currCell;
                                adjCell->t_val = currCell->t_val + 1; // add 1 to time of ancestor as the cell is one step away
                                open_pq.push(adjCell); // add the updated cell to the priority queue
                            }
                        }
                     
                        else
                        {   // if the location has never been explored before
                            adjCell = new SearchCell(newx, newy); // create new cell if the location has never been explored before
                            adjCell->h_val = h_map[GETMAPINDEX(newx,newy, x_size, y_size)]; // use heuristic from backward search
                            adjCell->cost_val = (int) map[GETMAPINDEX(newx,newy,x_size,y_size)];
                            adjCell->parent = currCell;
                            adjCell->g_val = currCell->g_val + adjCell->cost_val;
                            adjCell->t_val = currCell->t_val + 1; // add 1 to time of ancestor as the cell is one step away
                            open_pq.push(adjCell);
                        }
                    }
                }
            }
        }
        mexPrintf("Goal found after  %d\n", elapsed_time);
        // if the goal state is reached earlier then wait at the same location
        for (int i=0; i <= elapsed_time; i++)
        {
            currIdx = GETTIMEINDEX(currCell->x_pos, currCell->y_pos, currCell->t_val + i, x_size, y_size, target_steps);
            actionXY.first = currCell->x_pos;
            actionXY.second = currCell->y_pos;
            action_map[currIdx] = actionXY;
            mexPrintf("A star for loop \n");
        }
        
        // Backtrack from found position to robot
        parentCell = currCell->parent;
        while (parentCell)
        {
            currIdx = GETTIMEINDEX(parentCell->x_pos, parentCell->y_pos, parentCell->t_val, x_size, y_size, target_steps);
            actionXY.first = currCell->x_pos;
            actionXY.second = currCell->y_pos;
            currCell = parentCell;
            action_map[currIdx] = actionXY;
            parentCell = currCell->parent;
                        mexPrintf("A star after goal while loop \n");
        }
        
        mexPrintf("A star Complete \n");
    }
  
    //Action to return at every time step
    idx = GETTIMEINDEX(robotposeX,robotposeY, timeCount, x_size, y_size, target_steps);
    action_ptr[0] = (action_map[idx]).first;
    action_ptr[1] = (action_map[idx]).second;
    timeCount++;
}

// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
        
{
    
    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Six input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }
    
    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);
    
    if(targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
                "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;
    
    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if(targetpose_M != 1 || targetpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
                "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;   
}
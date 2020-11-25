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
#define MAP_IN                  prhs[0]
#define ROBOT_IN                prhs[1]
#define TARGET_TRAJ             prhs[2]
#define TARGET_POS              prhs[3]
#define CURR_TIME               prhs[4]
#define COLLISION_THRESH        prhs[5]


/* Output Arguments */
#define ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define MAX(A, B)   ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B)   ((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

using namespace std;
using namespace std::chrono;

class greaterG
{
    public:
        int operator() (const SearchCell* cl, const SearchCell* cr) const // cl: left cr: right
        {
            // this acts as greater in pq so we create a min-heap
            return (cl->g_val) > (cr->g_val);
        }
};

static void planner(
        double* map,
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
    
    static int idx; // target point and new neighbor index on trajectory
    // maps for actions
    static unordered_map<int, int> actionMapX;
    static unordered_map<int, int> actionMapY;

    priority_queue<SearchCell*, vector<SearchCell*>, greaterG> open_pq;
    unordered_map<int,SearchCell*> visited; // map to check which nodes have been visited before but have not been fully expanded yet (not found the optimal f value yet)
    unordered_map<int,bool> closed; // cell index and bool parameter to see if cell is visited already
    if (curr_time == 0) // Do all the planning at the first time step
    {   
        SearchCell* newCell = new SearchCell();
        newCell->x_pos = robotposeX; 
        newCell->y_pos = robotposeY;
        newCell->g_val = 0;
        open_pq.push(newCell);
        visited[GETMAPINDEX(newCell->x_pos, newCell->y_pos, x_size, y_size)] = newCell; 

        SearchCell* currCell,*interceptCell,*parentCell,*adjCell; // pointers for cells to explore
        int newIdx, currIdx, idx; // indices of cells to explore

        // Foraward Search
        while(!open_pq.empty())
        {
            currCell = open_pq.top();
            open_pq.pop();
            currIdx = GETMAPINDEX(currCell->x_pos, currCell->y_pos, x_size, y_size);
            if (closed[currIdx]){ continue;} // check if the cell is already in closed list
            closed[currIdx] = true; // add popped cell to the closed list

            for (int dir = 0; dir < NUMOFDIRS; dir++)
            {
                // Loop over all 8 successors and the stationary positions
                int newx = currCell->x_pos + dX[dir];
                int newy = currCell->y_pos + dY[dir];
                
                // Check if the adjacent cell is valid and inside the map
                if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
                {
                    if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
                    {
                        newIdx = GETMAPINDEX(newx, newy, x_size, y_size);
                        if (closed[newIdx]){continue;}

                        if (visited[newIdx])// if the cell has been visited before but hasn't been popped yet, see if g value can be updated
                        {    
                            adjCell = visited[newIdx];
                            if(adjCell->g_val > currCell->g_val + adjCell->cost_val)
                            {
                                adjCell->g_val = currCell->g_val + adjCell->cost_val;
                                adjCell->t_val = currCell->t_val + 1; // add 1 to time of ancestor as the cell is one step away
                                adjCell->parent = currCell;
                                open_pq.push(adjCell); // add the updated cell to the priority queue
                            }
                        }

                        else
                        { // if the location has never been explored before
                            adjCell = new SearchCell();  // create new cell if the location has never been explored before
                            adjCell->x_pos = newx;
                            adjCell->y_pos = newy;
                            adjCell->parent = currCell; 
                            adjCell->t_val = currCell->t_val + 1; // add 1 to time of ancestor as the cell is one step away
                            adjCell->cost_val = (int)map[GETMAPINDEX(newx,newy,x_size,y_size)];
                            adjCell->g_val = currCell->g_val + adjCell->cost_val;
                            visited[newIdx] = adjCell;
                            open_pq.push(adjCell);
                        }
                    }
                }
            }
        }

        // Find the location to intercept the target
        for (int i = 0; i < target_steps; i++)
        {
            currIdx = GETMAPINDEX(target_traj[i], target_traj[i+target_steps], x_size, y_size);
            currCell = visited[currIdx];

            if (i > currCell->t_val) 
            {
                interceptCell = currCell;
            }
        }
    
        currIdx = GETMAPINDEX(interceptCell->x_pos, interceptCell->y_pos, x_size, y_size);
        currCell = interceptCell;
        actionMapX[currIdx] = currCell->x_pos;
        actionMapY[currIdx] = currCell->y_pos;
        parentCell = currCell->parent;
        
        // Backtrack from found position to robot
        while(parentCell)
        {
            idx = GETMAPINDEX(parentCell->x_pos, parentCell->y_pos , x_size, y_size);
            actionMapX[idx] = currCell->x_pos;
            actionMapY[idx] = currCell->y_pos;
            currCell = parentCell;
            parentCell = currCell->parent;
        }
        open_pq = priority_queue <SearchCell*, vector<SearchCell*>, greaterG>(); // reset it
        visited.clear();
        closed.clear();
    }

    idx = GETMAPINDEX(robotposeX, robotposeY, x_size, y_size);
    action_ptr[0] = actionMapX[idx];
    action_ptr[1] = actionMapY[idx];

    return;
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
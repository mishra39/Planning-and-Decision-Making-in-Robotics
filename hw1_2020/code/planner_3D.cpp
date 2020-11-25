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

#define NUMOFDIRS 8 // add one extra direction

using namespace std;
using namespace std::chrono; 


class greaterG{
    public:
        int operator() (const SearchCell* cl, const SearchCell* cr) // cl: left cr: right
        {
            // this acts as greater in pq so we create a min-heap
            return (cl->g_val > cr->g_val);
        }
};

class greaterH{
    public:
        int operator() (const SearchCell* cl, const SearchCell* cr) // cl: left cr: right
        {
            // this acts as greater in pq so we create a min-heap
            return (cl->h_val > cr->h_val);
        }
};

//  Find the heuristic values of all the cells
/******Heuristic Value here will be the same for the optimal g-value as we are doing a backward search*********************/
unordered_map<int,int> h_calc(double* map, int collision_thresh, double* target_traj, int x_size, int y_size,int target_steps)
{
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    priority_queue<SearchCell* , vector<SearchCell*>, greaterG > open_pq;
    unordered_map<int,bool> closed; // cell index and bool parameter to see if cell is visited already
    unordered_map<int,SearchCell*> open_map; // map to check which nodes have been visited before but have not been fully expanded yet (not found the optimal f value yet)
    unordered_map<int,int> h_map; // Map to store the heuristic values from the backward Search
    int targetIdx, newIdx; // target point and new neighbor index on trajectory
    SearchCell* targetCell, *newCell; // Pointer to the cell of the point on trajectory

    // Add all the trajectory points to the open list
    for (int i = 0; i < target_steps; i++)
    {
        targetIdx = GETMAPINDEX(target_traj[i], target_traj[i+target_steps], x_size, y_size);  // get the idx/key of the target point
        targetCell = new SearchCell(target_traj[i], target_traj[i+target_steps]);  // Create a Searchcell pointer using current position
        targetCell->g_val = 0; // Initiate the root/goal with zero for backward search
        targetCell->cost_val =(int) map[targetIdx]; // Cost of the current position in the map
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

                if (((int)map[newIdx] >= 0) && ((int)map[newIdx] < collision_thresh)) // check to ensure that the cell is collision free and inside the map
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

    static unordered_map<int,int> action_mapX;
    static unordered_map<int,int> action_mapY;
    int idx;

    int timeCount = 0;
    if (curr_time == 0)
    {
        priority_queue<SearchCell* , vector<SearchCell*>, greaterH> open_pq;
        unordered_map<int,bool> closed; // cell index and bool parameter to see if cell is visited already
        unordered_map<int,SearchCell*> visited; // map to check which nodes have been visited before but have not been fully expanded yet (not found the optimal f value yet)
        unordered_map<int,int> h_map; // Map to store the heuristic values from the backward Search
        int currX, currY, currIdx, adjX, adjY, adjIdx; // target point and new neighbor index on trajectory
        int elapsed_time, currCelltime; // time elapsed so far
        SearchCell *currCell, *parentCell, *adjCell,*startCell; // Pointers to the current, neighbor, and previous cells
        static int tot_steps;
        time_t begin,end; // start and end time of planning
        begin = time(nullptr);

        // Backward Dijsktra search call to find heuristic for all cells on the map
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
                    break;
                }

                for (int dir = 0; dir < 9; dir++)
                { 
                    // Loop over all 8 successors and the stationary positions
                    int newx = currCell->x_pos + dX[dir];
                    int newy = currCell->y_pos + dY[dir];
                    adjIdx = GETTIMEINDEX(newx,newy, currCell->t_val + 1 ,x_size, y_size, target_steps); // currCelltime + 1 is the time to reach the neighbor
                    if (( (int)map[adjIdx] >= 0) && ((int)map[adjIdx] < collision_thresh)) // check to ensure that the cell is collision free and inside the map
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

        // if the goal state is reached earlier then wait at the same location
        for (int i=0; i <= elapsed_time; i++)
        {
            currIdx = GETTIMEINDEX(currCell->x_pos, currCell->y_pos, currCell->t_val + i, x_size, y_size, target_steps);
            action_mapX[currIdx] = currCell->x_pos;
            action_mapY[currIdx] = currCell->y_pos;
        }

        // Backtrack from found position to robot
        parentCell = currCell->parent;
        while (parentCell)
        {
            currIdx = GETTIMEINDEX(parentCell->x_pos, parentCell->y_pos, parentCell->t_val, x_size, y_size, target_steps);
            action_mapX[currIdx] = currCell->x_pos;
            action_mapY[currIdx] = currCell->y_pos;
            currCell = parentCell;
            parentCell = currCell->parent;
        }
        
    }

    //Action to return at every time step
    idx = GETTIMEINDEX(robotposeX,robotposeY, timeCount, x_size, y_size, target_steps);
    action_ptr[0] = action_mapX[idx];
    action_ptr[1] = action_mapX[idx];
    timeCount++;
}
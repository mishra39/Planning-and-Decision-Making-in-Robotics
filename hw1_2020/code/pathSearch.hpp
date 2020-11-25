#pragma once
#include <math.h>
#include<bits/stdc++.h> 
#include <queue>
#include <chrono>
#include <mex.h>
#include <unordered_map>
#include <time.h>
#include <iostream>
#include "SearchCell.hpp"

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
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

class pathSearch
{
private:
    double* map;
    int collision_thresh;
    int x_size;
    int y_size;
    int robotposeX;
    int robotposeY;
    int target_steps;
    double* target_traj;
    int targetposeX;
    int targetposeY;
    int curr_time;
    double* action_ptr;
public:
    pathSearch(double *map, int collision_thresh, int x_size, int y_size, int robotposeX, int robotposeY, int target_steps,
            double* target_traj, int targetposeX, int targetposeY, int curr_time, double* action_ptr)
    {
        this->map = map;
        this->collision_thresh = collision_thresh;
        this->x_size = x_size;
        this->y_size = y_size;
        this->robotposeX = robotposeX;
        this->robotposeY = robotposeY;
        this->target_steps = target_steps;
        this->target_traj = target_traj;
        this->targetposeX = targetposeX;
        this->targetposeY = targetposeY;
        this->curr_time = curr_time;
        this->action_ptr = action_ptr;
    }
     // 8-connected grid
    int dX[8] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[8] = {-1,  0,  1, -1,  1, -1, 0, 1};

    SearchCell* currCell,*interceptCell,*parentCell,*adjCell;
    priority_queue<SearchCell*, vector<SearchCell*>, greaterG> open_pq;
    unordered_map<int,SearchCell*> visited; // map to check which nodes have been visited before but have not been fully expanded yet (not found the optimal f value yet)
    unordered_map<int,bool> closed; // cell index and bool parameter to see if cell is visited already
    
    // maps for actions
    unordered_map<int, int> actionMapX;
    unordered_map<int, int> actionMapY;    
    int newx, newy, newIdx, currIdx, idx;

    void initNewCell();   
    void computePath();
    //void backTrack();
    ~pathSearch();
};

void pathSearch::initNewCell()
{
    SearchCell* newCell = new SearchCell();
    newCell->x_pos = this->robotposeX; 
    newCell->y_pos = this->robotposeY;
    newCell->g_val = 0;
    open_pq.push(newCell);
    visited[GETMAPINDEX(newCell->x_pos, newCell->y_pos, x_size, y_size)] = newCell;
}

void pathSearch::computePath()
{
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
                if ((newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size) && ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))
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
        while(parentCell){
            idx = GETMAPINDEX(parentCell->x_pos, parentCell->y_pos , x_size, y_size);
            actionMapX[idx] = currCell->x_pos;
            actionMapY[idx] = currCell->y_pos;
            currCell = parentCell;
            parentCell = currCell->parent;
        }
}

pathSearch::~pathSearch()
{
    this->open_pq = priority_queue <SearchCell*, vector<SearchCell*>, greaterG>(); // reset it
    this->visited.clear();
    this->closed.clear();
    this->actionMapX.clear();
    this->actionMapY.clear();
}
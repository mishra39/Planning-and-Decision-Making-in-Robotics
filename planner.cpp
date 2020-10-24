/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <iostream>
#include <random>
#include <vector>
#include <chrono>
#include <algorithm>
#include <bits/stdc++.h>
#include <map>
#include <queue>
#include "mex.h"

using namespace std;

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm (should be the same as the one used in runtest.m)
#define LINKLENGTH_CELLS 10

typedef struct {
  int X1, Y1;
  int X2, Y2;
  int Increment;
  int UsingYIndex;
  int DeltaX, DeltaY;
  int DTerm;
  int IncrE, IncrNE;
  int XIndex, YIndex;
  int Flipped;
} bresenham_param_t;


void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size)
{
    double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params)
{
  params->UsingYIndex = 0;

  if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
    (params->UsingYIndex)++;

  if (params->UsingYIndex)
    {
      params->Y1=p1x;
      params->X1=p1y;
      params->Y2=p2x;
      params->X2=p2y;
    }
  else
    {
      params->X1=p1x;
      params->Y1=p1y;
      params->X2=p2x;
      params->Y2=p2y;
    }

   if ((p2x - p1x) * (p2y - p1y) < 0)
    {
      params->Flipped = 1;
      params->Y1 = -params->Y1;
      params->Y2 = -params->Y2;
    }
  else
    params->Flipped = 0;

  if (params->X2 > params->X1)
    params->Increment = 1;
  else
    params->Increment = -1;

  params->DeltaX=params->X2-params->X1;
  params->DeltaY=params->Y2-params->Y1;

  params->IncrE=2*params->DeltaY*params->Increment;
  params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
  params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

  params->XIndex = params->X1;
  params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y)
{
  if (params->UsingYIndex)
    {
      *y = params->XIndex;
      *x = params->YIndex;
      if (params->Flipped)
        *x = -*x;
    }
  else
    {
      *x = params->XIndex;
      *y = params->YIndex;
      if (params->Flipped)
        *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params)
{
  if (params->XIndex == params->X2)
    {
      return 0;
    }
  params->XIndex += params->Increment;
  if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
    params->DTerm += params->IncrE;
  else
    {
      params->DTerm += params->IncrNE;
      params->YIndex += params->Increment;
    }
  return 1;
}



int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
		   int x_size,
 		   int y_size)

{
	bresenham_param_t params;
	int nX, nY; 
    short unsigned int nX0, nY0, nX1, nY1;

    //printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
    
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

    //printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
            return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
		   int x_size, int y_size)
{
    double x0,y0,x1,y1;
    int i;
    
 	//iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
    y1 = 0;
	for(i = 0; i < numofDOFs; i++)
	{
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
				return 0;
	}    
    return 1;
}

#define armDOF 5
struct Node
{
    int idx;
    double* jointAng;
    vector<Node*>* adjNodes; // adjacent nodes/neighbors
    vector<double>* adjNodesDist; 
    Node* parent;
    double f_val;
    double g_val;
    double cost;
    double h_val;
};
struct greaterF
  {
    int operator() (const Node* cl, const Node* cr) const // cl: left cr: right
    {
        // this acts as greater in pq so we create a min-heap
        return (cl->f_val) > (cr->f_val);
    }
  };
// Generate random arm configuration
void randomConfig(double** config_arr)
{
  random_device rd;
  default_random_engine gen(rd());
  uniform_real_distribution<double> distribution(0, 2*PI);

  for (int i = 0 ; i < 5; i++)
  {
    double pose = distribution(gen);
    (*config_arr)[i] = pose;
  }
}
bool sortbysec(const pair<Node*,double> &a, 
              const pair<Node*,double> &b) 
{ 
    return (a.second < b.second); 
} 
static void findNeighbors(vector<Node*>* graph, vector<Node*>* nearestKNodes,
  vector<double>* nearestKDist, double* config, double nnRad) // i represents the total number of nodes in the roadmap so far
{

  // traverse the graph and compute distance between nodes
    int j=0;
    double dist = 0;
    //printf("Starting Neighbor Search \n");
    for (j = 0; j < graph->size(); j++)
    {
      double dist = 0;
      Node* adjNode = (*graph)[j];
      for (int i = 0; i < armDOF; i++)
      {
        dist += pow((adjNode->jointAng[i] - config[i]),2);
      }

      dist  = sqrt(dist);
      if (dist <= nnRad)
      {
        //printf("neighbor found \n");
        nearestKNodes->push_back(adjNode);
        nearestKDist->push_back(dist);
      }
    }/*
      for (int k = 0; k < armDOF; k++)
        {
          dist +=  pow(fabs(adjNode->jointAng[k] - currConfig[k]),2); 
        }
      nearestK->push_back(make_pair(adjNode,dist));
    }
    if (j)
    {
      sort(nearestK->begin(), nearestK->end(), sortbysec);
      nearestK->erase(nearestK->begin()+kNN, nearestK->end()); // keep top k values
    }*/
}

bool isEdgeValid(double neighborDist, double* currNodeAng, double* adjNodeAng, double *map, int x_size, int y_size)
{
  int numofsamples = (int)((neighborDist)/(PI/20));
  double* newConfig_arr = (double*) malloc(armDOF * sizeof(double));
  for (int ff=1; ff <= numofsamples; ff++)
  {
    for(int kk = 0; kk < armDOF; kk++)
    {
      newConfig_arr[kk] = adjNodeAng[kk] + ((double)(ff)*(PI/20))*((currNodeAng[kk] - adjNodeAng[kk])/neighborDist);
    }
    if(!IsValidArmConfiguration(newConfig_arr, armDOF, map, x_size, y_size))
    {
      //printf("The nearest nodes have an obstacle in between \n");
      return false;
    }
  }
  return true;
}

// checks if the nearest node is on the same cluster, i.e. d is reachable from s
bool hasPath(Node *s, Node *d, int V)
{
  if (s->idx == d->idx)
  {
    return true;
  }

  // Mark all the vertices as not visited
  bool *visited = new bool[V+2]; // V+1 because we start at index 1 and last index will for goal node
  for (int i = 0; i <= V+1; i++)
  {
    visited[i] = false;
  }

  // Create a queue for BFS
  list<Node*> queue;

  // Mark the current node as visited and add it to the queue
  visited[s->idx] = true;
  //printf("Marked Node %d as visited \n" , s->idx);
  queue.push_back(s);

  while(!queue.empty())
  { // Dequeue a vertex
    s = queue.front();
    queue.pop_front();

    Node* neighbor; 
    // Find all the adjacent vertices and mark them as visited and enqueue them
    for (int i = 0; i< s->adjNodes->size(); i++)
    {
      neighbor  = (*(s->adjNodes))[i];
      if (neighbor->idx == d->idx) // if this node is the destination return true
      {
       // printf("Path already exists\n");
        delete[] visited;
        return true;
      }
      // BFS
      if (visited[neighbor->idx] == false)
      {
       // printf("Pushing node %d to queue \n", (*it)->idx);
        visited[neighbor->idx] = true;
        queue.push_back(neighbor);
      }
    }
  }
//  printf("Checked all Nodes and no path found\n");
  delete[] visited;
  return false;
}

double heuristicCalc(double* nodeConfig, double* goalConfig) // computes the distance between two configurations
{
  double heuristic = 0;
  for (int s=0; s<armDOF; s++)
  {
    heuristic += pow((nodeConfig[s] - goalConfig[s]),2);
  }
  heuristic  = sqrt(heuristic);
  return heuristic;
}
static void prmPlanner(double*	map,
		  int x_size,
 		  int y_size,
      double* armstart_anglesV_rad,
      double* armgoal_anglesV_rad,
      int numofDOFs
      ,double*** plan,
      int* planlength
    )
{
    int V = 10000;
    double kNNrad = 0.0;
    int i = 0;
    double epsilon = PI/4;

    vector<Node*>* graph = new vector<Node*>();
    double *currConfig;
    printf("Start Map Creation \n");
   
    while (i < V)
    {
      vector<Node*>* nearestKNodes = new vector<Node*>();
      vector<double>* nearestKDist = new vector<double>();
      currConfig = (double*) malloc(sizeof(double)*armDOF);
      Node* currNode = (Node*) malloc(sizeof(Node));
      Node* adjNode;
      randomConfig(&currConfig);
      if (!IsValidArmConfiguration(currConfig, armDOF, map, x_size, y_size))
      {
        continue;
      }
      i++;
      currNode->jointAng = currConfig;
      currNode->adjNodes = new vector<Node*>();
      currNode->adjNodesDist = new vector<double>();
      currNode->idx = i;
      graph->push_back(currNode);
      printf("Iteration %d of %d \n", i, V);
      kNNrad =  epsilon;
      findNeighbors(graph, nearestKNodes, nearestKDist,currConfig, kNNrad);
      
      for (int ss=0; ss < nearestKNodes->size(); ss++)
      {
        adjNode = (*nearestKNodes)[ss];
        double neighborDist = (*nearestKDist)[ss];
        if (!isEdgeValid(neighborDist,currConfig, adjNode->jointAng, map,x_size, y_size))
        {
         // printf("Edge not Valid \n");
          continue;
        }
        if (hasPath(currNode,adjNode,V))
        {
          continue;
        }
        currNode->adjNodes->push_back(adjNode);
        currNode->adjNodesDist->push_back(neighborDist);
        adjNode->adjNodes->push_back(currNode);
        adjNode->adjNodesDist->push_back(neighborDist);
      }
      free(nearestKNodes);
      free(nearestKDist);
    }
    printf("While Loop Complete \n");
    Node* initNode = (Node*) malloc(sizeof(Node));
    Node* goalNode = (Node*) malloc(sizeof(Node));
    double* startConfig = (double*) malloc(sizeof(double)*armDOF);
    double* goalConfig  = (double*) malloc(sizeof(double)*armDOF);
    for (int ang=0; ang < armDOF; ang++)
    {
      startConfig[ang] = armstart_anglesV_rad[ang];
      goalConfig[ang]  = armgoal_anglesV_rad[ang];
    }
    initNode->jointAng = startConfig;
    goalNode->jointAng = goalConfig;
    initNode->idx = 0;
    goalNode->idx = V + 1;
    initNode->adjNodes = new vector<Node*>();
    initNode->adjNodesDist = new vector<double>();
    goalNode->adjNodes = new vector<Node*>();
    goalNode->adjNodesDist = new vector<double>();
    graph->push_back(initNode);
    graph->push_back(goalNode);
    vector<Node*>* closestNodes = new vector<Node*>();
    vector<double>* closestDist = new vector<double>();
    Node* neighborNode;
    findNeighbors(graph, closestNodes, closestDist,armstart_anglesV_rad, epsilon);
    for (int ss=0; ss < closestNodes->size(); ss++)
    {
      neighborNode = (*closestNodes)[ss];
      double neighborDist = (*closestDist)[ss];
      if (!isEdgeValid(neighborDist,armstart_anglesV_rad, neighborNode->jointAng, map,x_size, y_size))
      {
       // printf("Edge not Valid \n");
        continue;
      }
      if (hasPath(initNode,neighborNode,V))
      {
        //printf("Path already exists to start node \n");
        continue;
      }
      printf("Start Node connected \n");
      initNode->adjNodes->push_back(neighborNode);
      initNode->adjNodesDist->push_back(neighborDist);
      neighborNode->adjNodes->push_back(initNode);
      neighborNode->adjNodesDist->push_back(neighborDist);
    }
    
    vector<Node*>* closestNodesGoal = new vector<Node*>();
    vector<double>* closestDistGoal = new vector<double>();
    
    findNeighbors(graph, closestNodesGoal, closestDistGoal,armgoal_anglesV_rad, epsilon*10);
    for (int ss=0; ss < closestNodesGoal->size(); ss++)
    {
      neighborNode = (*closestNodesGoal)[ss];
      double neighborDist = (*closestDistGoal)[ss];
      if (!isEdgeValid(neighborDist,armgoal_anglesV_rad, neighborNode->jointAng, map,x_size, y_size))
      {
        //printf("Edge not Valid \n");
        continue;
      }
      if (hasPath(goalNode,neighborNode,V))
      {
        //printf("Path already exists to goal node \n");
        continue;
      }
      printf("Goal Node connected \n");
      goalNode->adjNodes->push_back(neighborNode);
      goalNode->adjNodesDist->push_back(neighborDist);
      neighborNode->adjNodes->push_back(goalNode);
      neighborNode->adjNodesDist->push_back(neighborDist);
    }

    if (hasPath(initNode, goalNode,V))
    {
      printf("Path created from start to goal node \n");
    }
    // Start Search
    priority_queue<Node*, vector<Node*>, greaterF> open_pq;
    unordered_map<int, bool> visited;
    unordered_map<int, bool> closed;
    Node* currNode, *adjNode, *parentNode;
    list<Node*>finalPath; // indices of shortest path
    double searchWt = 5;
    // initialize and push start node to open list
    initNode->g_val = 0;
    initNode->cost = 0;
    initNode->parent= NULL;
    initNode->h_val = heuristicCalc(initNode->jointAng, armgoal_anglesV_rad);
    visited[0] = true;
    open_pq.push(initNode);
    
    // Expand graph
    while(!open_pq.empty())
    {
      currNode = open_pq.top();
      open_pq.pop();
      if (closed[currNode->idx])
      {
        continue;
      }
      closed[currNode->idx] = true;

      // if currNode is goal then break
      if (currNode == goalNode)
      {
        printf("Goal found \n");
        break;
      }

      for (int h=0; h < currNode->adjNodes->size(); h++)
      {
        // if the node has been expanded
          if (closed[(*(currNode->adjNodes))[h]->idx])
          {
            continue;
          }

          // check for g value update
          if (visited[(*(currNode->adjNodes))[h]->idx])
          {
            adjNode = (*(currNode->adjNodes))[h];
            if (adjNode->g_val > currNode->g_val + adjNode->cost)
            {
              adjNode->g_val = currNode->g_val + adjNode->cost;
              adjNode->f_val = adjNode->g_val + searchWt*adjNode->h_val;
              adjNode->parent = currNode;
              open_pq.push(adjNode);
            }
          }

          else 
          {
            adjNode = (*(currNode->adjNodes))[h];
            adjNode->cost = (*(currNode->adjNodesDist))[h];
            adjNode->g_val = currNode->g_val + adjNode->cost;
            adjNode->h_val = heuristicCalc(adjNode->jointAng,armgoal_anglesV_rad);
            adjNode->f_val = adjNode->g_val + searchWt*adjNode->h_val;
            adjNode->parent = currNode;
            visited[adjNode->idx] = true;
            open_pq.push(adjNode);
          }
      }
    }
    parentNode = currNode->parent;
    while(1)
    {
      if (currNode->idx > 0)
      {
        finalPath.push_front(currNode);
        currNode = currNode->parent;
      }
      else
      {
        finalPath.push_front(currNode);
        printf("Backtrack complete \n");
        break;
      }
    }
    printf("Size of final path before popping  %d \n", finalPath.size());
    *planlength = finalPath.size();
    *plan = (double**) malloc((*planlength)*sizeof(double*));
    for (int m = 0; m < *planlength; m++){
        (*plan)[m] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(int j = 0; j < numofDOFs; j++){
            (*plan)[m][j] = (*(finalPath.front())).jointAng[j];
        }
        finalPath.pop_front();
    }    
    if (finalPath.empty())
    {
      printf("All nodes removed from final path list \n");
    }
/*
    bool* visited  = (bool*) malloc(sizeof(bool)*(V+2));
    for (int hh=0; hh<=V+1; hh++)
    {
      visited[hh] = false;
    }
    queue<Node*> q;
    q.push(initNode);
    visited[initNode->idx] = true;
    Node* currNode;
    Node* adjNode;
    
    while (!q.empty())
    {
      currNode = q.front();
      q.pop();
      //printf("Popped index %d \n", currNode->idx);
      visited[currNode->idx] = true;

      if (currNode->idx == goalNode->idx)
      {
        printf("Goal Found \n");
        break;
      }
      for (int ll=0;ll<currNode->adjNodes->size(); ll++)
      {
        double dist = (*(currNode->adjNodesDist))[ll];
        adjNode = (*(currNode->adjNodes))[ll];
        if (visited[adjNode->idx]== false)
        {
          visited[adjNode->idx]== true;
          adjNode->parent = currNode;
          q.push(adjNode);
        }
      }
    }
    list<Node*> finalPath;
    currNode = goalNode;
    Node* parentNode;
    parentNode = currNode->parent;
    //*plan = (double***) malloc((V+1)*sizeof(double*));
    while(1)
    {
      if (currNode->idx > 0)
      {
        finalPath.push_front(currNode);
        currNode = currNode->parent;
      }
      else
      {
        finalPath.push_front(currNode);
        printf("Backtrack complete \n");
        break;
      }
      
    }

    *plan = (double**) malloc((finalPath.size())*sizeof(double*));
    for (int m = 0; m < finalPath.size(); m++){
        (*plan)[m] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(int j = 0; j < numofDOFs; j++){
            (*plan)[m][j] = (*(finalPath.front())).jointAng[j];
        }
        finalPath.pop_front();
    }    
    *planlength = finalPath.size();
*/
    for (int kk=0; kk < graph->size();kk++)
    {
      free((*graph)[kk]->jointAng);
      free((*graph)[kk]->adjNodes);
      free((*graph)[kk]);
    }

    free(closestDist);
    free(closestDistGoal);
    
  return;
}


Node* findRRTNeighbor(vector<Node*>* graph,
  double &nearestDist, double* config)
{
  double dist = 0;
  nearestDist = INT_MAX;
  Node* nearestNode;
  Node* neighborNode;
  for (int i = 0; i < graph->size(); i++)
  {
    neighborNode = (*graph)[i];
    for (int i = 0; i < armDOF; i++)
    {
      dist += pow(neighborNode->jointAng[i] - config[i],2);
    }

    dist = sqrt(dist);
    if (dist < nearestDist)
    {
     // printf("RRT closest node found \n");
      nearestDist = dist;
      nearestNode = (*graph)[i];
    }
  }
  return nearestNode;
}


static void rrtPlanner(double*	map,
		  int x_size,
 		  int y_size,
      double* armstart_anglesV_rad,
      double* armgoal_anglesV_rad,
      int numofDOFs
      ,double*** plan,
      int* planlength
     )
{
    int V = 17000;
    int i = 0;
    double epsilon = PI/4;

    vector<Node*>* graph = new vector<Node*>();
    
    double *currConfig;
    printf("Start RRT Map Creation \n");
    Node* initNode = (Node*) malloc(sizeof(Node));
    Node* goalNode;// = (Node*) malloc(sizeof(Node));
    double* startConfig = (double*) malloc(sizeof(double)*armDOF);
    double* goalConfig  = (double*) malloc(sizeof(double)*armDOF);
    for (int ang=0; ang < armDOF; ang++)
    {
      startConfig[ang] = armstart_anglesV_rad[ang];
      goalConfig[ang]  = armgoal_anglesV_rad[ang];
    }
    initNode->jointAng = startConfig;
    initNode->idx = 0;
    initNode->adjNodes = new vector<Node*>();
    initNode->adjNodesDist = new vector<double>();
    graph->push_back(initNode);
    //graph->push_back(goalNode);
    
    while (i < V)
    {
      //printf("new attempt \n");
      currConfig = (double*) malloc(sizeof(double)*armDOF);
      Node* nearestNode;//  = (Node*) malloc(sizeof(Node));
      Node* newVertex =  (Node*) malloc(sizeof(Node));
      newVertex->adjNodes = new vector<Node*>();
      newVertex->adjNodesDist = new vector<double>();
      double nearestDist = 0;
      double* newConfig = (double*) malloc(sizeof(double)*armDOF);
      randomConfig(&currConfig);
      //printf("Checking IsValidArmConfiguration for random config\n");
      if (!IsValidArmConfiguration(currConfig, armDOF, map, x_size, y_size))
      {
        continue;
      }
      // Find nearest node
      //printf("Neighbor search complete \n");
      nearestNode = findRRTNeighbor(graph, nearestDist,currConfig);
      //printf("Neighbor search complete \n");
      // Extend the configuration towards q_rand by a distance epsilon
      for (int s = 0; s < armDOF; s++)
      {
        newConfig[s] = (currConfig[s] - nearestNode->jointAng[s]);
      }
      //printf("Assigned newconfig \n");
      double newConfigMag = 0;
      for (int s = 0; s < armDOF; s++)
      {
        newConfigMag += pow(newConfig[s],2);
      }

      //printf("newconfig computed \n");
      newConfigMag = sqrt(newConfigMag);
      for (int s = 0; s < armDOF; s++)
      {
        newConfig[s] = nearestNode->jointAng[s] + ((newConfig[s] / newConfigMag) * epsilon) ;
      }
      
      newVertex->jointAng = newConfig;
      //printf("Assigned newVertex \n");
      // Check if the new configuration is valid and the edge is collision free
      if ((!IsValidArmConfiguration(newVertex->jointAng, armDOF, map, x_size, y_size)))
        //|| (!isEdgeValid(epsilon,nearestNode->jointAng,newVertex->jointAng , map,x_size, y_size))
      
      {
        //printf("Checking validity \n");
        continue;
      }
      i++;
      newVertex->idx = i;
      printf("Iteration %d of %d \n", i, V);
      graph->push_back(newVertex);
      if (hasPath(nearestNode,newVertex,V))
      {
       // printf("Path already exists \n");
        continue;
      }
      //printf("path does not exist \n");
      nearestNode->adjNodes->push_back(newVertex);
      nearestNode->adjNodesDist->push_back(nearestDist);
      newVertex->adjNodes->push_back(nearestNode);
      newVertex->adjNodesDist->push_back(nearestDist);
      printf("Pushed adjacent nodes\n");
      // Check if new vertex is q_rand or in the goal region
      double goalDist = 0;
      for (int s=0; s<armDOF; s++)
      {
        goalDist += pow(armgoal_anglesV_rad[s] - newVertex->jointAng[s],2);
      }
      
      if (sqrt(goalDist) <= epsilon)
      {
        printf("Goal Region Reached \n");
        goalNode = newVertex;
        break;
      }
      else if (i == V)
      {
        goalNode = newVertex;
      }
    }
    if (hasPath(initNode, goalNode,V))
    {
      printf("Path created from start to goal node \n");
    }

    // Start Search
    priority_queue<Node*, vector<Node*>, greaterF> open_pq;
    unordered_map<int, bool> visited;
    unordered_map<int, bool> closed;
    Node* currNode, *adjNode, *parentNode;
    list<Node*>finalPath; // indices of shortest path
    double searchWt = 5;
    // initialize and push start node to open list
    initNode->g_val = 0;
    initNode->cost = 0;
    initNode->parent= NULL;
    initNode->h_val = heuristicCalc(initNode->jointAng, armgoal_anglesV_rad);
    visited[0] = true;
    open_pq.push(initNode);

    // Expand graph
    while(!open_pq.empty())
    {
      currNode = open_pq.top();
      open_pq.pop();
      if (closed[currNode->idx])
      {
        continue;
      }
      closed[currNode->idx] = true;

      // if currNode is goal then break
      if (currNode == goalNode)
      {
        printf("Goal found \n");
        break;
      }

      for (int h=0; h < currNode->adjNodes->size(); h++)
      {
      // if the node has been expanded
        if (closed[(*(currNode->adjNodes))[h]->idx])
        {
          continue;
        }

        // check for g value update
        if (visited[(*(currNode->adjNodes))[h]->idx])
        {
          adjNode = (*(currNode->adjNodes))[h];
          if (adjNode->g_val > currNode->g_val + adjNode->cost)
          {
            adjNode->g_val = currNode->g_val + adjNode->cost;
            adjNode->f_val = adjNode->g_val + searchWt*adjNode->h_val;
            adjNode->parent = currNode;
            open_pq.push(adjNode);
          }
        }

        else 
        {
          adjNode = (*(currNode->adjNodes))[h];
          adjNode->cost = (*(currNode->adjNodesDist))[h];
          adjNode->g_val = currNode->g_val + adjNode->cost;
          adjNode->h_val = heuristicCalc(adjNode->jointAng,armgoal_anglesV_rad);
          adjNode->f_val = adjNode->g_val + searchWt*adjNode->h_val;
          adjNode->parent = currNode;
          visited[adjNode->idx] = true;
          open_pq.push(adjNode);
        }
      }
    }
    
    parentNode = currNode->parent;
    while(1)
    {
      if (currNode->idx > 0)
      {
        finalPath.push_front(currNode);
        currNode = currNode->parent;
      }
      else
      {
        finalPath.push_front(currNode);
        printf("Backtrack complete \n");
        break;
      }
    }
    printf("Size of final path before popping  %d \n", finalPath.size());
    *planlength = finalPath.size();
    *plan = (double**) malloc((*planlength)*sizeof(double*));
    for (int m = 0; m < *planlength; m++){
        (*plan)[m] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(int j = 0; j < numofDOFs; j++){
            (*plan)[m][j] = (*(finalPath.front())).jointAng[j];
        }
        finalPath.pop_front();
    }    
    if (finalPath.empty())
    {
      printf("All nodes removed from final path list \n");
    }
    
    for (int kk=0; kk < graph->size();kk++)
    {
      free((*graph)[kk]->jointAng);
      free((*graph)[kk]->adjNodes);
      free((*graph)[kk]);
    }
  
  return;
}
static void planner(
		   double*	map,
		   int x_size,
 		   int y_size,
           double* armstart_anglesV_rad,
           double* armgoal_anglesV_rad,
	   int numofDOFs,
	   double*** plan,
	   int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
    //for now just do straight interpolation between start and goal checking for the validity of samples

    double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
            distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
    }
    int numofsamples = (int)(distance/(PI/20));
    if(numofsamples < 2){
        printf("the arm is already at the goal\n");
        return;
    }
    *plan = (double**) malloc(numofsamples*sizeof(double*));
    int firstinvalidconf = 1;
    for (i = 0; i < numofsamples; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(j = 0; j < numofDOFs; j++){
            (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
        }
        if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) && firstinvalidconf)
        {
            firstinvalidconf = 1;
            printf("ERROR: Invalid arm configuration!!!\n");
        }
    }    
    *planlength = numofsamples;
    
    return;
}

//prhs contains input parameters (3): 
//1st is matrix with all the obstacles
//2nd is a row vector of start angles for the arm 
//3nd is a row vector of goal angles for the arm 
//plhs should contain output parameters (2): 
//1st is a 2D matrix plan when each plan[i][j] is the value of jth angle at the ith step of the plan
//(there are D DoF of the arm (that is, D angles). So, j can take values from 0 to D-1
//2nd is planlength (int)
void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[])
     
{ 
    
    /* Check for proper number of arguments */    
    if (nrhs != 4) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Four input arguments required."); 
    } else if (nlhs != 2) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 
        
    /* get the dimensions of the map and the map matrix itself*/     
    int x_size = (int) mxGetM(MAP_IN);
    int y_size = (int) mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the start and goal angles*/     
    int numofDOFs = (int) (MAX(mxGetM(ARMSTART_IN), mxGetN(ARMSTART_IN)));
    if(numofDOFs <= 1){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "it should be at least 2");         
    }
    double* armstart_anglesV_rad = mxGetPr(ARMSTART_IN);
    if (numofDOFs != MAX(mxGetM(ARMGOAL_IN), mxGetN(ARMGOAL_IN))){
        	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "numofDOFs in startangles is different from goalangles");         
    }
    double* armgoal_anglesV_rad = mxGetPr(ARMGOAL_IN);
 
    //get the planner id
    int planner_id = (int)*mxGetPr(PLANNER_ID_IN);
    if(planner_id < 0 || planner_id > 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidplanner_id",
                "planner id should be between 0 and 3 inclusive");         
    }
    
    //call the planner
    double** plan = NULL;
    int planlength = 0;
    
    //you can may be call the corresponding planner function here
    //if (planner_id == RRT)
    //{
    //    plannerRRT(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
    //}
    
    //dummy planner which only computes interpolated path
    //planner(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength); 
    
    printf("planner returned plan of length=%d\n", planlength); 
    if (planner_id == PRM)
    {
      printf("Running PRM Planner \n");
      prmPlanner(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad,numofDOFs, &plan,&planlength);
    }

    if (planner_id == RRT)
    {
      printf("Running RRT Planner \n");
      rrtPlanner(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad,numofDOFs, &plan,&planlength);
    }
    /* Create return values */
    if(planlength > 0)
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)planlength, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);        
        //copy the values
        int i,j;
        for(i = 0; i < planlength; i++)
        {
            for (j = 0; j < numofDOFs; j++)
            {
                plan_out[j*planlength + i] = plan[i][j];
            }
        }
    }
    else
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);
        //copy the values
        int j;
        for(j = 0; j < numofDOFs; j++)
        {
                plan_out[j] = armstart_anglesV_rad[j];
        }     
    }
    PLANLENGTH_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL); 
    int* planlength_out = (int*) mxGetPr(PLANLENGTH_OUT);
    *planlength_out = planlength;

    return;
    
}
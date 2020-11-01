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

bool isEdgeValidRRT(double neighborDist, double* Xnearest, double* Xnew,double* prevConfig, double *map, int x_size, int y_size)
{
  int numofsamples = (int)((neighborDist)/(PI/40));
  printf("isEdgeValidRRT: Total Number of Samples %d \n", numofsamples);
  double* newConfig_arr = (double*) malloc(armDOF * sizeof(double));
  
  //printf("isEdgeValidRRT: Starting Edge Validation \n");
  for (int ff=1; ff <= numofsamples; ff++)
  {
    for(int kk = 0; kk < armDOF; kk++)
    {
      newConfig_arr[kk] = Xnearest[kk] + ((double)(ff)*(PI/40))*((Xnearest[kk] - Xnew[kk])/neighborDist);
    }
    //printf("isEdgeValidRRT: moved config \n");
    if(!IsValidArmConfiguration(newConfig_arr, armDOF, map, x_size, y_size))
    {
      printf("The nearest nodes have an obstacle in between \n");
      return false;
    }
    if (ff > 5)
    {
     // printf("isEdgeValidRRT: Assigning the last valid moved config \n");
      prevConfig = (double*) malloc(sizeof(double)*armDOF);
      for (int k =0; k < armDOF; k++)
      {
        //printf("isEdgeValidRRT:Assigning %.4f to previous Config\n",newConfig_arr[k]);
        prevConfig[k] = newConfig_arr[k];
      }
    }
    
  }
  printf("isEdgeValidRRT: Edge is fully valid \n");
  return true;
}

// checks if the nearest node is on the same cluster, i.e. d is reachable from s
bool hasPath(Node *s, Node *d, int V)
{
  if (s->idx == d->idx)
  {
    return true;
  }
  //printf("function hasPath: Indices do not match.\n");
  // Mark all the vertices as not visited
  bool *visited = new bool[V+2]; // V+1 because we start at index 1 and last index will for goal node
  for (int i = 0; i <= V+1; i++)
  {
    visited[i] = false;
  }
  //printf("function hasPath: Marked all visited as false\n");

  // Create a queue for BFS
  list<Node*> queue;

  // Mark the current node as visited and add it to the queue
  visited[s->idx] = true;
  queue.push_back(s);
  //int size  = s->adjNodes->size();
  //printf("Size of adjacent nodes %d \n ", size);
  //printf("function hasPath: Marked Node %d as visited. Starting while loop \n" , s->idx);

  while(!queue.empty())
  { // Dequeue a vertex
    s = queue.front();
    queue.pop_front();
    
    Node* neighbor; 
    //printf("function hasPath: Popped Element with idx %d \n", s->idx);
    // Find all the adjacent vertices and mark them as visited and enqueue them
    for (int i = 0; i < s->adjNodes->size(); i++)
    {
      //printf("function hasPath: Exploring neighbors \n");
      neighbor  = (*(s->adjNodes))[i];
      if (neighbor->idx == d->idx) // if this node is the destination return true
      {
       //printf("function hasPath: Path already exists\n");
        delete[] visited;
        return true;
      }
      // BFS
      if (visited[neighbor->idx] == false)
      {
        visited[neighbor->idx] = true;
        queue.push_back(neighbor);
      }
    }
  }
  //printf("Checked all Nodes and no path found\n");
  delete[] visited;
  //printf("Returning now \n");
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

double distanceCalc(double *configA, double* configB)
{
  double dist = 0;
  for (int i = 0; i < armDOF; i++)
  {
    dist += pow(configA[i]-configB[i],2);
  }
  return (sqrt(dist));
}
void goalbiasConfig(double* goalConfig, double regionSize, double** randCOnfig)
{
  random_device rd;
  default_random_engine gen(rd());
  for (int i = 0 ; i < 5; i++)
  {
    uniform_real_distribution<double> distribution(goalConfig[i] - regionSize, goalConfig[i] + regionSize);
    double pose = distribution(gen);
    (*randCOnfig)[i] = pose;
  }
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
    if (!IsValidArmConfiguration(armstart_anglesV_rad, armDOF, map,x_size,y_size))
    {
      printf("Invalid Start Position \n");
      return;
    }
    if (!IsValidArmConfiguration(armgoal_anglesV_rad, armDOF, map,x_size,y_size))
    {
      printf("Invalid Goal Position \n");
      return;
    }
    int V = 10000;
    double kNNrad = 0.0;
    int i = 0;
    double epsilon = PI/4;

    vector<Node*>* graph = new vector<Node*>();
    double *currConfig;
    //printf("Start Map Creation \n");
   
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
      //printf("Iteration %d of %d \n", i, V);
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
    printf("Size of the graph created for PRM Planner: %d\n", graph->size());
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
        //printf("Backtrack complete \n");
        break;
      }
    }
    //printf("Size of final path before popping  %d \n", finalPath.size());
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
  //printf("Distance of the NN is : %.2f \n", nearestDist);

  return nearestNode;
}

// Extends by epsilon
void steer(double* Xnearest, double* Xrand, double* newConfig, double nearestDist, double epsilon)
{
  // Extend the configuration towards q_rand by a distance epsilon
  for (int s = 0; s < armDOF; s++)
  {
    newConfig[s] = (Xrand[s] - Xnearest[s]);
  }
  //printf("Assigned newconfig \n");
  double newConfigMag = 0;
  for (int s = 0; s < armDOF; s++)
  {
    newConfigMag += pow(newConfig[s],2);
  }

  //printf("newconfig computed \n");
  newConfigMag = sqrt(newConfigMag);
  if (nearestDist > epsilon)
  {
    for (int s = 0; s < armDOF; s++)
    {
      newConfig[s] = Xnearest[s] + ((newConfig[s] / newConfigMag) * epsilon) ;
    }
    //printf("Moved node by distance %.2f and epsilon is %.2f \n", distanceCalc(newConfig, Xnearest),epsilon);
  }
  
  else 
  {
    for (int j = 0; j < armDOF; j++)
    {
      newConfig[j] = Xrand[j];
    }
  }
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
    int V = 25000;
    int i = 0;
    double epsilon = PI/4;

    vector<Node*>* graph = new vector<Node*>();
    if (!IsValidArmConfiguration(armstart_anglesV_rad, armDOF, map,x_size,y_size))
    {
      printf("Invalid Start Position \n");
      return;
    }
    if (!IsValidArmConfiguration(armgoal_anglesV_rad, armDOF, map,x_size,y_size))
    {
      printf("Invalid Goal Position \n");
      return;
    }
    double *currConfig;
    printf("Start RRT Map Creation \n");
    Node* initNode = (Node*) malloc(sizeof(Node));
    Node* goalNode;// = (Node*) malloc(sizeof(Node));
    double* startConfig = (double*) malloc(sizeof(double)*armDOF);
    double* goalConfig  = (double*) malloc(sizeof(double)*armDOF);
    double* lastGoodConfig;
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
    
    while (i < V)
    {
      //printf("new attempt \n");
      currConfig = (double*) malloc(sizeof(double)*armDOF);
      if (i>100 && (i%100 ==0))
      {
        //printf("************************Creating Bias Goal for iteration %d ************************\n", i);
        goalbiasConfig(armgoal_anglesV_rad, epsilon/4, &currConfig);
      }
      else 
      {
        randomConfig(&currConfig);
      }
      if (!IsValidArmConfiguration(currConfig, armDOF, map,x_size,y_size))
      {
        continue;
      }
      Node* nearestNode;//  = (Node*) malloc(sizeof(Node));
      Node* newVertex =  (Node*) malloc(sizeof(Node));
      newVertex->adjNodes = new vector<Node*>();
      newVertex->adjNodesDist = new vector<double>();
      double nearestDist = 0;
      double* newConfig = (double*) malloc(sizeof(double)*armDOF);
      //printf("Checking IsValidArmConfiguration for random config\n");
      // Find nearest node
      //printf("Neighbor search complete \n");
      nearestNode = findRRTNeighbor(graph, nearestDist,currConfig);
      //printf("Neighbor search complete \n");
      // Extend the configuration towards q_rand by a distance epsilon
      if (nearestDist > epsilon)
      {
        for (int s = 0; s < armDOF; s++)
        {
          steer(nearestNode->jointAng, currConfig, newConfig, nearestDist, epsilon);// newConfig[s] = nearestNode->jointAng[s] + ((newConfig[s] / newConfigMag) * epsilon) ;
        }
        printf("Moved node by distance %.2f \n", distanceCalc(newConfig, nearestNode->jointAng));
      }

      double newDist = distanceCalc(newConfig,nearestNode->jointAng);
      lastGoodConfig = NULL;
      if (!isEdgeValidRRT(newDist,nearestNode->jointAng,newConfig,lastGoodConfig, map,x_size, y_size))
      {
        if (lastGoodConfig == NULL)
        {
          //printf("NULL returned from edge validation \n");
          continue;
        }
        for (int k = 0; k < armDOF; k++)
        {
          newConfig[k] = lastGoodConfig[k];
        }
      }
      if (!IsValidArmConfiguration(newConfig, armDOF, map,x_size,y_size))
      {
        continue;
      }
      newVertex->jointAng = newConfig;
      //printf("Assigned newVertex \n");
      // Check if the new configuration is valid and the edge is collision free
      i++;
      newVertex->idx = i;
      newVertex->parent = nearestNode;
      printf("Iteration %d of %d \n", i, V);
      graph->push_back(newVertex);
      nearestNode->adjNodes->push_back(newVertex);
      nearestNode->adjNodesDist->push_back(newDist);
      newVertex->adjNodes->push_back(nearestNode);
      newVertex->adjNodesDist->push_back(newDist);
      //printf("Pushed adjacent nodes\n");
      // Check if new vertex is q_rand or in the goal region
      double goalDist = 0;
      for (int s=0; s<armDOF; s++)
      {
        goalDist += pow(armgoal_anglesV_rad[s] - newVertex->jointAng[s],2);
      }
      
      if (sqrt(goalDist) <= epsilon/2)
      {
        printf("Goal Region Reached \n");
        goalNode = newVertex;
        break;
      }
      else if (i >= V-5)
      {
        double goalNearestDist = 0;
        goalNode = findRRTNeighbor(graph, goalNearestDist,armgoal_anglesV_rad);;
        printf("Max iterations reached \n");
        break;
      }
    }
    /*if (hasPath(initNode, goalNode,V))
    {
      printf("Path created from start to goal node \n");
    }*/

    Node* closestToGoal = goalNode;//findRRTNeighbor(graph, goalNearestDist,armgoal_anglesV_rad);
    // BackTrack
    list<Node*>finalPath;
    while(closestToGoal->parent != NULL)
    {
      
      finalPath.push_front(closestToGoal);
      if (closestToGoal->idx == 0)
      {
        break;
      }
      //printf("Searching for parents of idx  \n", closestToGoal->idx);
      closestToGoal = closestToGoal->parent;  
      //printf("Parent Successfully  found\n");
    }

    printf("Size of path is %d  and size of graph %d\n ", finalPath.size(), graph->size());
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

// Generate random index for nodes
int randomIdx(int startorGoal, int V)
{
  random_device rd;
  mt19937_64 generator(rd());
  if (startorGoal == 0)
  {
    uniform_int_distribution<int> dist{1, (int) (V/2)};
    return (dist(generator));
  }

  else
  {
    uniform_int_distribution<int> dist{(int)((V/2) +1), V-1};
    return (dist(generator));
  }
}
void addEdge (Node** newVertex, Node **nearestNode, double newConfigMag, int V)
{
  if (!hasPath(*newVertex,*nearestNode,V))
  {
    printf("adding Edge--------------------------------------------- \n");
    (*newVertex)->adjNodes->push_back(*nearestNode);
    (*newVertex)->adjNodesDist->push_back(newConfigMag);
    (*nearestNode)->adjNodes->push_back(*newVertex);
    (*nearestNode)->adjNodesDist->push_back(newConfigMag);
  }
}

double* extend(double* targetConfig, double* nearestNodeConfig, double &closestDist, double epsilon)
{
  double* newConfig;
  while (closestDist > epsilon)
  {
    printf("Extending \n");
    for (int j = 0; j < armDOF; j++)
    {
      newConfig[j] = nearestNodeConfig[j] + epsilon*((targetConfig[j] - nearestNodeConfig[j])/closestDist);
    }
    closestDist -= epsilon;
    nearestNodeConfig = newConfig; 
  }
  return newConfig;
}
static void rrtConnectPlanner(double*	map,
		  int x_size,
 		  int y_size,
      double* armstart_anglesV_rad,
      double* armgoal_anglesV_rad,
      int numofDOFs
      ,double*** plan,
      int* planlength
     )
{
    if (!IsValidArmConfiguration(armstart_anglesV_rad, armDOF, map,x_size,y_size))
    {
      printf("Invalid Start Position \n");
      return;
    }
    if (!IsValidArmConfiguration(armgoal_anglesV_rad, armDOF, map,x_size,y_size))
    {
      printf("Invalid Goal Position \n");
      return;
    }
    *plan = NULL;
    *planlength = 0;
    int V = 25000;
    int startGraphIdx = 0; // counter for node indices in the start tree
    int goalGraphIdx = 0; // counter for node indices in the goal tree
    double epsilon = PI/4;
    int startOrGoal = 0; // 0 to start
    int i=0;
    vector<Node*>* graphStart = new vector<Node*>();
    vector<Node*>* graphGoal = new vector<Node*>();
    
    double *randConfig;
    double* newConfigA;
    double* newConfigB;
    double* lastGoodConfig;
    printf("Start RRT Connect Map Creation \n");
    Node* initNode = (Node*) malloc(sizeof(Node));
    Node* goalNode = (Node*) malloc(sizeof(Node));
    initNode->adjNodes = new vector<Node*>();
    goalNode->adjNodes = new vector<Node*>();
    initNode->adjNodesDist = new vector<double>();
    goalNode->adjNodesDist = new vector<double>();

    Node* startGraphNode, *goalGraphNode;
    double* startConfig = (double*) malloc(sizeof(double)*armDOF);
    double* goalConfig  = (double*) malloc(sizeof(double)*armDOF);
    
    for (int ang=0; ang < armDOF; ang++)
    {
      startConfig[ang] = armstart_anglesV_rad[ang];
      goalConfig[ang]  = armgoal_anglesV_rad[ang];
    }

    initNode->jointAng = startConfig;
    initNode->idx = 0;
    initNode->parent = NULL;
  
    goalNode->jointAng = goalConfig;
    goalNode->idx = V; // Change after graph creation
    graphStart->push_back(initNode);
    graphGoal->push_back(goalNode);
    unordered_map<int,bool> uniqueId; // checks to make sure unique id is created everytime
    uniqueId[0] = true;
    uniqueId[V] = true;

    vector<Node*>* currGraph = graphStart; // pointer to current graph for forward search
    vector<Node*>* graphB = graphGoal; // pointer to current graph for forward search
    Node* currNode;
    Node* backNode;

    while(i<V)
    {
      randConfig = (double*) malloc(sizeof(double)*armDOF);
      if (i>100 && (i%20 ==0))
      {
        printf("************************Creating Bias Goal for iteration %d ************************\n", i);
        goalbiasConfig(armgoal_anglesV_rad, epsilon, &randConfig);
      }
      else 
      {
        randomConfig(&randConfig);
      }
      
      if (!IsValidArmConfiguration(randConfig, armDOF, map, x_size, y_size))
      {
        continue;
      }
      Node* nearestNode;
      double nearestDist=0;
      newConfigA = (double*) malloc(sizeof(double)*armDOF);
      nearestNode = findRRTNeighbor(currGraph, nearestDist,randConfig); // Find nearest node
      // move only by epsilon towards the sampled config
      steer(nearestNode->jointAng,randConfig,newConfigA,nearestDist, epsilon);
      for (int k =0; k < armDOF; k++)
      {
        printf("After steer function, newConfigA is %.4f\n",newConfigA[k]);
      }
      double newConfigDist = distanceCalc(nearestNode->jointAng, newConfigA);
      lastGoodConfig = NULL;
      if (!isEdgeValidRRT(newConfigDist,nearestNode->jointAng,newConfigA,lastGoodConfig,map,x_size, y_size))
      {
        if (lastGoodConfig == NULL)
        {
          printf("NULL returned from edge validation \n");
          i++;
          continue;
        }
        for (int k = 0; k < armDOF; k++)
        {
          newConfigA[k] = lastGoodConfig[k];
        }
      }
      newConfigDist = distanceCalc(nearestNode->jointAng, newConfigA); // update the distance if needed
      if (!IsValidArmConfiguration(newConfigA, armDOF, map,x_size,y_size))
      {
        continue;
      }
      printf("Distance between newConfig A and its nearest Node is: %.2f \n", newConfigDist);
      currNode = (Node*) malloc(sizeof(Node));
      currNode->adjNodes = new vector<Node*>();
      currNode->adjNodesDist = new vector<double>();
      currNode->jointAng = newConfigA;
      printf("assigned newConfigA to currNode \n");
      int tempIdx = 0;
      while(uniqueId[tempIdx]==true)
      {
        tempIdx = randomIdx(startOrGoal, V);
      }
      // Add vertex to graphA
      currNode->idx = tempIdx;
      currNode->parent = nearestNode;
      currNode->adjNodes->push_back(nearestNode);
      currNode->adjNodesDist->push_back(newConfigDist);
      currGraph->push_back(currNode);
      // Extend towards to newConfigA from graphB for as long as possible
      nearestDist=0;
      newConfigB = (double*) malloc(sizeof(double)*armDOF);
      double* tempConfigNearestNode = (double*) malloc(sizeof(double)*armDOF);
      nearestNode = findRRTNeighbor(graphB, nearestDist,newConfigA); // Find nearest node
      for (int l = 0;  l < armDOF; l++)
      {
        tempConfigNearestNode[l] = nearestNode->jointAng[l]; // temp variable for connect function
      }
      double distToMove = nearestDist;
      double steerDist = epsilon/2; // distance to extend on each iteration
      while(1)
      {
        steer(tempConfigNearestNode,newConfigA,newConfigB,nearestDist, steerDist);// divide epsilon to make sure the targetConfig is not missed
        newConfigDist = distanceCalc(tempConfigNearestNode, newConfigB);
        lastGoodConfig = NULL;
        if (!isEdgeValidRRT(newConfigDist,tempConfigNearestNode,newConfigB,lastGoodConfig,map,x_size, y_size))
        {
          if (lastGoodConfig != NULL)
          {
            for (int k =0; k < armDOF; k++)
            {
              newConfigB[k] = lastGoodConfig[k];
              printf("Full Edge was not valid. The best case config returned was %.4f \n", lastGoodConfig[k]);
            }
          }
          else 
          {
            newConfigB = NULL;
            printf("Trapped Scenario. Returning NULL for newConfigB \n");
          }
          printf("breaking extend for tree B \n");
          break;
        }
        distToMove -= distanceCalc(tempConfigNearestNode, newConfigB); // calculate distance moved and subtract that from full distance
        if (distanceCalc(newConfigB, newConfigA) == 0.00)
        {
          /*for (int k =0; k < armDOF; k++)
          {
            newConfigB[k] = newConfigA[k];
          }*/
          newConfigB  = newConfigA;
          printf("Graphs Connected \n");
          break;
        }
        // break if not enough steering room left
        if (distToMove < steerDist)
        {
          printf("Breaking: Not enough steering room!!!!!! \n");
          break;
        }

        for (int l = 0;  l < armDOF; l++)
        {
          tempConfigNearestNode[l] = newConfigB[l]; // temp variable for connect function
        }
      }

      if (newConfigB != NULL)
      {
        // Push the new node after extending to graphB
        backNode = (Node*) malloc(sizeof(Node));
        backNode->adjNodes = new vector<Node*>();
        backNode->adjNodesDist = new vector<double>();

        backNode->jointAng = newConfigB;
        int tempIdx = 0;
        while(uniqueId[tempIdx]==true)
        {
          tempIdx = randomIdx(!startOrGoal, V);
        }
        newConfigDist = distanceCalc(nearestNode->jointAng, newConfigB);
        // Add vertex to graphB
        backNode->idx = tempIdx;
        backNode->parent = nearestNode;
        backNode->adjNodes->push_back(nearestNode);
        backNode->adjNodesDist->push_back(newConfigDist);
        graphB->push_back(backNode);
        if (newConfigB  == newConfigA)
        {
          printf("Path found from start to Goal \n");
          break;
        }
        i++;
        printf("i : %d \n",i);
      }
      // Swap trees
      printf("Swapping \n");
      currGraph = (startOrGoal==0) ? graphGoal : graphStart;
      graphB = (startOrGoal==1) ? graphGoal : graphStart;
      startOrGoal = (startOrGoal==0) ? 1 : 0;
    }
    

    currNode = (currGraph == graphStart) ? currNode : backNode;
    backNode = (currGraph == graphStart) ? backNode : currNode;

    // Back track from currrent Node from graphA
    list<double*> finalPathA;
    list<double*> finalPathB;
    while (1)
    {
      if ((currNode->parent->idx == 0) || (currNode->parent->idx == V))
      {
        printf("Breaking before index %d \n", currNode->parent->idx);
        finalPathA.push_front(currNode->jointAng);
        finalPathA.push_front(currNode->parent->jointAng);
        break;
      }
      finalPathA.push_front(currNode->jointAng);
      currNode = currNode->parent;
    }
    
    while (1)
    {
      if ((backNode->parent->idx == 0) || (backNode->parent->idx == V))
      {
        printf("Breaking before index %d \n", backNode->parent->idx);
        finalPathB.push_back(backNode->jointAng);
        finalPathB.push_back(backNode->parent->jointAng);
        break;
      }
      finalPathB.push_front(backNode->jointAng);
      backNode = backNode->parent;
    }
    
    backNode = backNode->parent;
    int pathASize = finalPathA.size();
    int pathBSize = finalPathB.size(); 
    printf("Size of GraphStart is %d and size of graphGoal is: %d \n",graphStart->size(), graphGoal->size());
    printf("Length of path A: %d. Length of path B: %d \n",pathASize, pathBSize);
    
    
    *planlength = pathASize+pathBSize;
    printf("Size of path found %d\n", *planlength);

    *plan = (double**) malloc((*planlength)*sizeof(double*));
    for (int m = pathASize-1; m >=0; m--){
        double* tempConfig = finalPathA.front();
        finalPathA.pop_front();
        (*plan)[m] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(int j = 0; j < numofDOFs; j++){
            (*plan)[m][j] = tempConfig[j];
        }
        printf("index of m is : %d\n", m);
    }
    printf("Path A Assigned \n");
    for (int m = pathASize; m < (pathASize + pathBSize); m++){
        double* tempConfig = finalPathB.front();
        finalPathB.pop_front();
        (*plan)[m] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(int j = 0; j < numofDOFs; j++){
            (*plan)[m][j] = tempConfig[j];
        }
        printf("index of m in second loop is : %d\n", m);
    }
    printf("Path B Assigned \n");
    /*
    for (int f=0; f < graphStart->size(); f++)
    {
      free((*graphStart)[f]->jointAng);
      free((*graphStart)[f]->adjNodes);
      free((*graphStart)[f]);
    }
    for (int f=0; f < graphGoal->size(); f++)
    {
      free((*graphGoal)[f]->jointAng);
      free((*graphGoal)[f]->adjNodes);
      free((*graphGoal)[f]);
    }*/
}

static void rrtStarPlanner(
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
  if (!IsValidArmConfiguration(armstart_anglesV_rad, armDOF, map,x_size,y_size))
  {
    printf("Invalid Start Position \n");
    return;
  }
  if (!IsValidArmConfiguration(armgoal_anglesV_rad, armDOF, map,x_size,y_size))
  {
    printf("Invalid Goal Position \n");
    return;
  }
  int V = 25000;
  int i = 0;
  double epsilon = PI/4;
  vector<Node*>* graph = new vector<Node*>();
  Node* initNode = (Node*) malloc(sizeof(Node));
  double *randConfig;
  double* startConfig = (double*) malloc(sizeof(double)*armDOF);
  double* goalConfig  = (double*) malloc(sizeof(double)*armDOF);
  for (int ang=0; ang < armDOF; ang++)
  {
    startConfig[ang] = armstart_anglesV_rad[ang];
    goalConfig[ang]  = armgoal_anglesV_rad[ang];
  }
  initNode->jointAng = startConfig;
  initNode->idx = 0;
  initNode->parent = NULL;
  initNode->cost = 0;
  initNode->g_val = 0;
  initNode->adjNodes = new vector<Node*>();
  initNode->adjNodesDist = new vector<double>();
  graph->push_back(initNode);
  printf("Star......RRT Star \n");
  while (graph->size() < V)
  {
    randConfig = (double*) malloc(sizeof(double)*armDOF);
    if (i>100 && (i%20 ==0))
    {
      //printf("************************Creating Bias Goal for iteration %d ************************\n", i);
      goalbiasConfig(armgoal_anglesV_rad, epsilon/2, &randConfig);
    }
    else 
    {
      randomConfig(&randConfig);
    }
    if (!IsValidArmConfiguration(randConfig, armDOF, map,x_size,y_size))
    {
      continue;
    }

    // Find the closest neighbor
    Node* nearestNode;
    double nearestDist = 0;
    nearestNode = findRRTNeighbor(graph,nearestDist, randConfig);
    //printf("Nearest dist after neighbor search %.2f\n",nearestDist);
    // Steer function
    double* newConfig = (double*) malloc(sizeof(double)*armDOF);
    steer(nearestNode->jointAng, randConfig, newConfig, nearestDist, epsilon);
    if (!IsValidArmConfiguration(newConfig, armDOF, map, x_size,y_size))
    {
      continue;
    }
    double newConfigDist = distanceCalc(newConfig, nearestNode->jointAng);
    if (!isEdgeValid(newConfigDist,newConfig, nearestNode->jointAng, map, x_size, y_size))
    {
      continue;
    }

    // Sample nodes in the specified region
    double gamma = 2.0;
    int rad = min((gamma/armDOF)*(log(graph->size())/ graph->size()), epsilon);
    vector<Node*>* closestNodes = new vector<Node*>();
    vector<double>* closestDist = new vector<double>();
    findNeighbors(graph, closestNodes, closestDist, newConfig, rad); // find neighbors of new config
    Node* adjNode;
    Node* Xnew = (Node*) malloc(sizeof(Node));
    Xnew->adjNodes = new vector<Node*>();

    Node* bestNode; // node with the least cost to the new config
    double adjNodeDist = 0;
    bestNode = nearestNode;
    double minCost = nearestNode->cost + newConfigDist; //variable to compare closest
    for (int ss=0; ss < closestNodes->size(); ss++)
    {
      adjNode = (*closestNodes)[ss];
      adjNodeDist = (*closestDist)[ss];
      if (!isEdgeValid(adjNodeDist, newConfig, adjNode->jointAng, map, x_size, y_size))
      {
        continue;
      }
      if (minCost > (adjNode->cost + adjNodeDist))
      {
        minCost = adjNode->cost + adjNodeDist;
        bestNode  = adjNode;
      }
    }
    i++;
    Xnew->parent = bestNode;
    Xnew->cost = minCost;
    Xnew->idx = i;
    Xnew->jointAng = newConfig;
    graph->push_back(Xnew);

    // Improve paths to all the vertices
    for (int ss=0; ss < closestNodes->size(); ss++)
    {
      adjNode = (*closestNodes)[ss];
      adjNodeDist = (*closestDist)[ss];
      if (!isEdgeValid(adjNodeDist, Xnew->jointAng, adjNode->jointAng, map, x_size, y_size))
      {
        continue;
      }

      if ((Xnew->cost + adjNodeDist) < adjNode->cost)
      {
        adjNode->parent = Xnew;
        adjNode->cost = Xnew->cost + adjNodeDist;
        Xnew->adjNodes->push_back(Xnew);
      }
    }

    if (distanceCalc(newConfig, armgoal_anglesV_rad) < epsilon/4)
    {
      printf("Goal Found \n");
      break;
    }
  }
  // If the goal is not sampled, find the path from closest to goal point
  double goalNearestDist = 0;
  Node* closestToGoal = findRRTNeighbor(graph, goalNearestDist,armgoal_anglesV_rad);
  printf("Size of final graph %d\n", graph->size());
  // BackTrack
  list<Node*>finalPath;
  while(closestToGoal->parent != NULL)
  {
    finalPath.push_front(closestToGoal);
    closestToGoal = closestToGoal->parent;  
  }
  printf("Size of path is %d  and index of final node %d\n ", finalPath.size(), closestToGoal->idx);
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
  for (int f=0; f < graph->size(); f++)
  {
    free((*graph)[f]->jointAng);
    free((*graph)[f]->adjNodes);
    free((*graph)[f]);
  }
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
    
    //printf("planner returned plan of length=%d\n", planlength); 
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

    if (planner_id == RRTCONNECT)
    {
      printf("RRT Connect Planner \n");
      rrtConnectPlanner(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad,numofDOFs, &plan,&planlength);
    }

    if (planner_id == RRTSTAR)
    {
      printf("RRT Star Planner \n");
      rrtStarPlanner(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad,numofDOFs, &plan,&planlength);
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
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
#define armDOF 5
using namespace std;

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
class SearchNode
{
public:
    int idx;
    double jointAng[armDOF];
    double g_val;
    double cost;
    double h_val;
    double f_val;
    SearchNode* parent;
  SearchNode()
  { };
  ~SearchNode() 
  {
  //delete this->parent;
  };
};
// roadmap (graph) for the configuration space
class roadMap:public SearchNode
{
private:
  double*	map;
  int x_size;
  int y_size;
  double* armstart_anglesV_rad;
  double* armgoal_anglesV_rad;
  int numofDOFs;
  double*** plan;
  int* planlength;
public:
  struct Node
    {
      int idx;
      double jointAng[armDOF];
    };
    struct greaterF
    {
      int operator() (const SearchNode* cl, const SearchNode* cr) const // cl: left cr: right
      {
          // this acts as greater in pq so we create a min-heap
          return (cl->f_val) > (cr->f_val);
      }
    };
  int V; // total verticies in the roadMap
  vector<pair<Node*,double>> adjList[200]; // Weighted graph adjacency list
  unordered_map<int,Node*> nodeMap;
  int kNN; // number of nearest neighbors
  Node *prmNode;
  roadMap(double *map, int x_size, int y_size, int V, int kNN, double* armstart_anglesV_rad, double* armgoal_anglesV_rad)
  {
    this->V = V;
    this->kNN = kNN;
    this->map = map;
    this->x_size = x_size;
    this->y_size = y_size;
    this->armstart_anglesV_rad = armstart_anglesV_rad;
    this->armgoal_anglesV_rad = armgoal_anglesV_rad;
    *plan = (double**) malloc(V*sizeof(double*));
  }

  void addEdge(Node* x,Node* y, double wt)
  {
    adjList[x->idx].push_back(make_pair(y,wt));
    adjList[y->idx].push_back(make_pair(x,wt));
  }
  
  double heuristicCalc(double* nodeConfig, double* goalConfig) // computes the distance between two configurations
  {
    double heuristic = 0;
    for (int s=0; s<armDOF; s++)
    {
      heuristic += fabs(nodeConfig[s] - goalConfig[s]);
    }
    return heuristic;
  }

// Generate random arm configuration
double* randomConfig()
{
  static double config_arr[armDOF];
  random_device rd;
  default_random_engine gen(rd());
  uniform_real_distribution<double> distribution(0, 2*PI);

  for (int i = 0 ; i < 5; i++)
  {
    double pose = distribution(gen);
    config_arr[i] = pose;
  }
 // printf("New Pose verification: %.2f , %.2f \n", newConfig[4], config_arr[4]);
  return config_arr;
}
  void createMap()
  {
    int i = 0;
    while (i < V)
    {
      double *newConfig_arr;
      newConfig_arr = randomConfig(); // find randomConfig, check validity and collision
      if (!IsValidArmConfiguration(newConfig_arr, armDOF, map, x_size, y_size))
      {
       // printf("Invalid Random Pose \n");
        continue;
      }
      i++; // Start appending with index 1 (index zero will be for start node)
      //printf("Valid Random Pose \n");
      prmNode  = new Node();
      prmNode->idx = i; // assign index to node
      //printf("Assigned Random Pose to new node\n");
      for (int s=0; s<armDOF; s++)
      {
        prmNode->jointAng[s] = newConfig_arr[s]; // node configuration
        //printf("Assigned Angles new node\n");
      }
      nodeMap[i] = prmNode;
      //printf("At %d we have %d \n",i,nodeMap.count(i));
      vector<pair<double,int>> nearestK;
      //printf("Finding Neighbors \n");
      nearestK = findNeighbors(prmNode,i);  // find k-nearest neighbors
      //printf("Neighbor search over \n");
      // connect to the k-nearest neighbors
      for (auto v:nearestK)
      { 
        //printf("Closest node returned at %d\n", v);
        // check if path already exists
        if (hasPath(prmNode,nodeMap[v.second]) == false ) // if a path does not exist, add an edge
        {
          printf("Adding Edge between %d and %d\n", prmNode->idx, nodeMap[v.second]->idx);
          addEdge(prmNode,nodeMap[v.second], v.first);
        }
        else 
        {
          printf("Path already exists between %d and %d\n", prmNode->idx, nodeMap[v.second]->idx);
        }
      }
    }
    prmNode = new Node();
    prmNode->idx = 0;
    for (int s=0; s<armDOF; s++)
    {
      prmNode->jointAng[s] = armstart_anglesV_rad[s]; // node configuration
    }
    nodeMap[0] = prmNode;
    
    vector<pair<double,int>> nearestK;
    nearestK = findNeighbors(prmNode,i);  // find k-nearest neighbors
    addEdge(prmNode,nodeMap[nearestK[0].second], nearestK[0].first);
    printf("Added Edge between start point and %d \n", nearestK[0].second);

    prmNode = new Node();
    prmNode->idx = V+1;
    for (int s=0; s<armDOF; s++)
    {
      prmNode->jointAng[s] = armgoal_anglesV_rad[s]; // node configuration
    }
    nodeMap[V+1] = prmNode;
    nearestK = findNeighbors(prmNode,i);  // find k-nearest neighbors
    addEdge(prmNode,nodeMap[nearestK[0].second], nearestK[0].first);
    printf("Added Edge between goal point and %d \n", nearestK[0].second);
  }

// checks if the nearest node is on the same cluster, i.e. d is reachable from s
bool hasPath(Node *s, Node *d)
{
  if (s->idx == d->idx)
  {
    return true;
  }

  // Mark all the vertices as not visited
  bool *visited = new bool[V+1]; // V+1 because we start at index 1 and last index will for goal node
  for (int i = 0; i <= V; i++)
  {
    visited[i] = false;
  }

  // Create a queue for BFS
  list<Node*> queue;

  // Mark the current node as visited and add it to the queue
  visited[s->idx] = true;
  //printf("Marked Node %d as visited \n" , s->idx);
  queue.push_back(s);

  // iterator for traversing all the neighbors of the vertex
  vector<pair<Node*, double>>::iterator it;

  while(!queue.empty())
  { // Dequeue a vertex
    s = queue.front();
    queue.pop_front();

    // Find all the adjacent vertices and mark them as visited and enqueue them
    for (it = adjList[s->idx].begin(); it != adjList[s->idx].end(); ++it)
    {
      if ((it->first->idx) == d->idx) // if this node is the destination return true
      {
        printf("Path already exists\n");
        delete[] visited;
        return true;
      }
      // BFS
      if (visited[(it->first->idx)] == false)
      {
       // printf("Pushing node %d to queue \n", (*it)->idx);
        visited[it->first->idx] = true;
        queue.push_back(it->first);
      }
    }
  }
//  printf("Checked all Nodes and no path found\n");
  delete[] visited;
  return false;
}
// returns the indices of the closest nodes
  vector<pair<double,int>> findNeighbors(Node* currNode, int i) // i represents the total number of nodes in the roadmap so far
  {
   // printf("Finding neighbors for node at %d \n", currNode->idx);
    priority_queue<pair<double,int>, vector<pair<double,int>>, greater<pair<double,int>>> kNearest;
    vector<pair<double,int>> closestK;

    // traverse the graph and compute distance between nodes
    for (int j = 0; j < i; j++)
    {
      double dist = 0;
      if (j!= currNode->idx && nodeMap.count(j)) // check if the vertex exists
      {
      //  printf("Computing distance between nodes at %d and %d \n", currNode->idx, nodeMap[j]->idx);
        for (int k = 0; k < armDOF; k++)
        {
          dist +=  fabs(currNode->jointAng[k] - nodeMap[j]->jointAng[k]); 
        }
        kNearest.push(make_pair(dist,j));
      }
    }

    if (kNearest.size()>=kNN)
    {
      for (int m=0; m < kNN; m++)
      {
        closestK.push_back(kNearest.top()); // push the index and weight of the node
       // printf("Distance of the node at %d is %.2f \n", kNearest.top().second, kNearest.top().first);
        kNearest.pop();
      }
    }

    else
    {
      while (!kNearest.empty())
      {
        closestK.push_back(kNearest.top()); // push the index of the node
       // printf("Distance of the node at %d is %.2f \n", kNearest.top().second, kNearest.top().first);
        kNearest.pop();
      }
    }
    return closestK;
  }

// A* to find shortest path
  void shortestPath()
  {
    priority_queue<Node*, vector<SearchNode*>, greaterF> open_list;
    unordered_map<int, bool> visited;
    unordered_map<int, bool> closed;
    SearchNode* startNode, *currNode, *adjNode, *parentNode;
    list<int>finalPath; // indices of shortest path

    vector<pair<Node*,double>>::iterator it;
    double searchWt = 5;
    // initialize and push start node to open list
    startNode = new SearchNode();
    startNode->idx = 0;
    visited[0] = true;
    startNode->g_val = 0;
    startNode->cost = 0;
    for (int s=0; s<armDOF; s++)
    {
      startNode->jointAng[s] = armstart_anglesV_rad[s];
    }

    startNode->h_val = heuristicCalc(armstart_anglesV_rad, armgoal_anglesV_rad); 
    open_list.push(startNode);
    // Expand graph
    while(!open_list.empty())
    {
      currNode = open_list.top();
      open_list.pop();
      printf("Index of popped Node: %d \n", currNode->idx);
      if (closed[currNode->idx]) {continue;}
      closed[currNode->idx] = true;
  
      // if currNode is goal then break
      if (currNode->idx == V+1)
      {
        printf("Goal found \n");
        break;
      }   
      // Explore the adjacent nodes
      for (it = adjList[currNode->idx].begin(); it != adjList[currNode->idx].end(); ++it)
      { 
        printf("Exploring Adjacent Node \n", it->first->idx);
        if (currNode->idx == it->first->idx) 
        {
          continue;
        }
        if (closed[it->first->idx]) {
          printf("Node already closed \n");
          continue;
          }; // if the state has been fully expanded

        if (visited[it->first->idx])
        {
          printf("Node has been visited before. Checking for g value update\n");
          adjNode = new SearchNode();
          if (adjNode->g_val > currNode->g_val + adjNode->cost) // update g value if not optimal
          {
            adjNode->g_val = currNode->g_val + adjNode->cost;
            adjNode->f_val = adjNode->g_val + searchWt*adjNode->h_val;
            adjNode->parent = currNode;
            open_list.push(adjNode);
          }
        }

        else // if the node has not been visited yet
        {
          printf("Expanding for the first time\n");
          adjNode = new SearchNode();
          adjNode->idx = it->first->idx;
          adjNode->cost = it->second; // weight of the edge is the distance between the node configurations
          adjNode->g_val = currNode->g_val + adjNode->cost;
          adjNode->h_val = heuristicCalc(it->first->jointAng, armgoal_anglesV_rad); 
          adjNode->f_val = adjNode->g_val + searchWt*adjNode->h_val;
          adjNode->parent = currNode;
          visited[adjNode->idx] = true;
          open_list.push(adjNode);
        }
      }
    }
    finalPath.push_front(currNode->idx); // push the goal node to the list
    // Backtrack from goal to start
    parentNode  = currNode->parent;
    while(parentNode) // loop until you reach the start goal, which has no parent
    {
      finalPath.push_front(parentNode->idx);
      currNode = parentNode;
      if (currNode->idx != 0)
      {
        parentNode = currNode->parent;
      }
      else
      {
        break;
      }
    }

    // Print final path
    for (auto nodeId:finalPath)
    {
      printf("Node %d -->  ", nodeId);
    }
    printf("\n");
  }
  ~roadMap();
};

roadMap::~roadMap()
{
  delete prmNode;
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
  int V = 150;// number of nodes in the roadMap
  int kNN = 5;// nearest neighbors for the search
  roadMap prm(map,x_size,y_size,V,kNN, armstart_anglesV_rad, armgoal_anglesV_rad);
  prm.createMap();
  prm.shortestPath();
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
    planner(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength); 
    
    printf("planner returned plan of length=%d\n", planlength); 
    
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
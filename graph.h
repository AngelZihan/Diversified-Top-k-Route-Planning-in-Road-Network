#ifndef GRAPH_H
#define	GRAPH_H
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <chrono>
#include "heap.h" 
#include <stdlib.h>
#include <unordered_map>
#include <unordered_set> 
#include <algorithm> 
#include <stack>
#include <queue>
//#include <boost/heap/fibonacci_heap.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind/bind.hpp>
#include "math.h"
#define PI acos(-1)

using namespace std;
using namespace benchmark; 

typedef struct COMPARENODE
{
	pair<int, int> pif;//ID, minCost
	bool operator() (const struct COMPARENODE& a, const struct COMPARENODE& b) const  
	{  
		return a.pif.second > b.pif.second; 
	} 
}compareNode;

struct Edge
{
	int ID1, ID2, length, edgeID, cost; 
};

struct PTNode
{
	int nodeID;
	unordered_map<int, int>	PTRoad;	//childrenID, roadID
	unordered_map<int, int> PTChildren; //childrenID, node Pos
};

class Graph
{
public:
	Graph(){}
	Graph(string filename)
	{
        readUSMap(filename);
	}

	//True: Only adjList and adjListMap are used
	//False: Also use R versions
	int nodeNum;
	int edgeNum;

	vector<vector<pair<int, int> > > adjList;		//neighborID, Distance
	vector<vector<pair<int, int> > > adjListR;
	vector<vector<pair<int, int> > > adjListEdge;	//neighbor,edgeID
	vector<vector<pair<int, int> > > adjListEdgeR;

	vector<vector<pair<int, int> > > adjListCost;		//neighborID, cost
	vector<vector<pair<int, int> > > adjListCostR;

	int countNum;
    int Node[20];
    int estList[20];
	//vector<int> estList;
    vector<vector<int> > dNode;


	vector<Edge> vEdge;
	vector<Edge> vEdgeR;
	int readCost(string filename);

    //prepare for the LCA calculation
    vector<int> EulerSeq;
    vector<int> rEulerSeq;
    vector<int> toRMQ;
    vector<vector<int>> RMQIndex;
    vector<int> makeRMQDFS(int p, vector<vector<int> >& vSPT, vector<int>& vSPTParent); 
    vector<vector<int>> makeRMQ(int p, vector<vector<int> >& vSPT, vector<int>& vSPTHeight, vector<int>& vSPTParent); 
    int LCAQuery(int _p, int _q, vector<vector<int> >& vSPT, vector<int>& vSPTHeight, vector<int>& vSPTParent);


	//Identify the ISO nodes
	vector<bool> vbISOF;	//forward
	vector<bool> vbISOB;	//backward
	vector<bool> vbISOU;	//F & B
	vector<bool> vbISO;		//F | B
	int ISONodes();
	int BFS(int nodeID, bool bF, vector<bool>& vbVisited);

	vector<pair<double, double> > vCoor;
	unordered_map<string, pair<int, int> > mCoor;
	double minX, minY, maxX, maxY;

	//lon, lat
	int readUSMap(string filename);

	//	void contractNode(int threshold);

    vector<int> vDijkstra(int ID1);
    int iBoundingAstar(int ID1, int ID2, unordered_set<int>& sRemovedNode, vector<int>& vPath, vector<int>& vPathEdge, int T);
    int Landmark(int ID1, int ID2);

    void SPT(int root, vector<int>& vSPTDistance, vector<int>& vSPTParent, vector<int>& vSPTHeight, vector<int>& vSPTParentEdge, vector<vector<int> >& vSPT);
    void rSPT(int root, vector<int>& vSPTDistance, vector<int>& vSPTParent, vector<int>& vSPTParentEdge, vector<vector<int> >& vSPT);//reverse SP Tree

    int cTKSPD(int ID1, int ID2, int k, vector<int>& kResults, double t, int& countNumber, int& popPath);
    int eKSPNew(int ID1, int ID2, int k, vector<int>& kResults, vector<vector<int> >& vkPath, double t, int& countNumber, int& popPath, vector<float>& sim, float &SimTime);
    int eKSPCompare(int ID1, int ID2, int k, vector<int>& kResults, vector<vector<int> >& vkPath, double t, int& countNumber, int& popPath, float& percentage, float &SimTime);
    int eKSPPrune(int ID1, int ID2, int k, vector<int>& kResults, vector<vector<int> >& vkPath, double t, int& countNumber, int& popPath);
    void FindRepeatedPath(vector<vector<int> >& vvPath);
};

#endif

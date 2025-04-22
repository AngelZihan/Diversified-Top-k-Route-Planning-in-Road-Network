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
    int readBeijingMapDirected(string filename);
    int readUSMap(string filename);
    int readUSCost(string filename);
    int readUSMapCost(string filename);
    int readTestCoor(string filename);
    int readTestMap(string filename);
    int readExampleMap(string filename);

    //test.cpp
    void testCSP(string filename);
    void SCPT(int root, vector<int>& vSPTDistance, vector<int>& vSPTCost, vector<int>& vSPTParent, vector<int>& vSPTParentEdge, vector<vector<int> >& vSPT, int C);
    void rCPT(int root, int ID1, int C, vector<int>& vrSPTCost, vector<int>& vrSPTDistance);

    //	void contractNode(int threshold);

    vector<int> Dijkstra(int ID1, int ID2);
    vector<int> vDijkstra(int ID1);
    int DijkstraPath(int ID1, int ID2, vector<int>& vPath, vector<int>& vPathEdge);
    int DijkstraPath2(int ID1, int ID2, unordered_set<int>& sRemovedNode, vector<int>& vPath, vector<int>& vPathEdge);
    int iBoundingAstar(int ID1, int ID2, unordered_set<int>& sRemovedNode, vector<int>& vPath, vector<int>& vPathEdge, int T);
    int AStar(int ID1, int ID2, vector<pair<double, double> >& vCoor);
    int AStarLandmark(int ID1, int ID2, vector<pair<double, double> >& vCoor);
    int AStarLandmarkPath(int ID1, int ID2, vector<int>& vPath, vector<int>& vPathEdge);
    int AStarPath(int ID1, int ID2, vector<int>& vPath, vector<int>& vPathEdge, string& city);
    int EuclideanDistance(int ID1, int ID2, vector<pair<double, double> >& vCoor);
    int EuclideanDistance_2(int ID1, int ID2, vector<pair<double, double> >& vCoor);
    int Landmark(int ID1, int ID2);
    int EuclideanDistanceAdaptive(int ID1, int ID2, int latU, int lonU);

    int Yen(int ID1, int ID2, int k, vector<int>& kResults, double t, int& countNumber, int& popPath);
    int YenReverse(int ID1, int ID2, int k, vector<int>& kResults, vector<vector<int> >& vkPath, double t, int& countNumber, int& popPath);
    int Yen2003(int ID1, int ID2, int k, vector<int>& kResults);
    int Pascoal(int ID1, int ID2, int k, vector<int>& kResults, vector<vector<int> >& vkPath);
    void SPT(int root, int ID2, vector<int>& vSPTDistance, vector<int>& vSPTParent, vector<int>& vSPTHeight, vector<int>& vSPTParentEdge, vector<vector<int> >& vSPT);

    int BestFirst(int ID1, int ID2, int k, vector<int>& kResults);
    int iBounding(int ID1, int ID2, int k, vector<int>& kResults);
    int cSPT(int ID1, int ID2, int k, vector<int>& kResults);
    int FindKSPD(int ID1, int ID2, int k, vector<int>& kResults, double t, int& countNumber, int& popPath);
    int FindKSPDNew(int ID1, int ID2, int k, vector<int>& kResults, double t, int& countNumber, int& popPath);
    int cSPTKSPD(int ID1, int ID2, int k, vector<int>& kResults, double t, int& countNumber, int& popPath);
    int cTKSPD(int ID1, int ID2, int k, vector<int>& kResults, double t, int& countNumber, int& popPath);
    int onePass(int ID1, int ID2, int k, vector<int>& kResults, double t, int& countNumber, int& popPath);
    int multiPass(int ID1, int ID2, int k, vector<int>& kResults, double t, int& countNumber, int& popPath);

    //int eKSPNew(int ID1, int ID2, int k, vector<int>& kResults, vector<vector<int> >& vkPath);
    //int cKSPNew(int ID1, int ID2, int k, vector<int>& kResults, vector<vector<int> >& vkPath);
    //CIKM2010 in cKSP
    typedef struct LABEL
    {
        int pre;
        int post;
        int parent;
    }label;
    int cKSP(int ID1, int ID2, int k, vector<int>& kResults, vector<vector<int> >& vkPath);
    void rSPT(int root, vector<int>& vSPTDistance, vector<int>& vSPTParent, vector<int>& vSPTParentEdge, vector<vector<int> >& vSPT);//reverse SP Tree
    void intervalLabel(int root, vector<vector<int> >& vSPT, vector<int>& vSPTParent, vector<label>& vLabel);
    int DijkstraSideCost(int ID1, int ID2, vector<vector<pair<int, int> > >& adjSideCost, vector<vector<pair<int, int> > >& adjSideRoad, vector<int>& vPath, vector<int>& vPathEdge);
    int DijkstraSideCostPrune(int ID1, int ID2, vector<vector<pair<int, int> > >& adjSideCost, vector<vector<pair<int, int> > >& adjSideRoad, vector<int>& vSPTParent, vector<int>& vSPTParentEdge, vector<int>& vSPTDistance, vector<label>& vLabel, vector<int>& vIgnoredNode, vector<int>& vPath, vector<int>& vPathEdge, bool& bE);

    //CSP
    pair<int, int> fKSPCSP(int ID1, int ID2, int C);
    void rCSPT(int root, vector<int>& vSPTDistance, vector<int>& vSPTCost, vector<int>& vSPTParent, vector<int>& vSPTParentEdge, vector<vector<int> >& vSPT);
    void rCostLB(int root, vector<int>& vSPTCostLB, vector<int>& vCostUB);
    void sideDist(vector<int>& vSPTDistance, vector<vector<pair<int, int> > >& adjSideDist, vector<vector<pair<int, int> > >& adjSideRoad, vector<Edge>& vSideEdge);
//	int DijkstraSideCostPruneCSP(int ID1, int ID2, vector<vector<pair<int, int> > >& adjSideDist, vector<vector<pair<int, int> > >& adjSideRoad, vector<int>& vSPTParent, vector<int>& vSPTParentEdge, vector<int>& vSPTDistance, vector<label>& vLabel, vector<int>& vIgnoredNode, vector<int>& vPath, vector<int>& vPathEdge, bool& bE);
    int DijkstraSideCostPruneCSP(int ID1, int ID2, vector<vector<pair<int, int> > >& adjSideDist, vector<vector<pair<int, int> > >& adjSideRoad, vector<int>& vSPTParent, vector<int>& vSPTParentEdge, vector<int>& vSPTDistance, vector<int>& vSPTCost, vector<label>& vLabel, vector<int>& vIgnoredNode, vector<int>& vPath, vector<int>& vPathEdge, bool& bE, int& subLength, int& subCost, vector<int>& vSPTCostLB, vector<map<int, int> >& vmSkyline, int& dTmp, int& cTmp);

    pair<int, int> cKSPCSP(int ID1, int ID2, int C);
    pair<int, int> Pulse(int ID1, int ID2, int C);
    int eKSP(int ID1, int ID2, int C);
    pair<int, int> skylineCSP(int ID1, int ID2, int C);
    void cD(int ID1, int ID2);
    void forwardCost(int ID1, int C, vector<int>& vFCost);


    int kspCSP(int ID1, int ID2, int C);

    void cKSPNew(int ID1, int ID2, int k, vector<int>& kResults, vector<vector<int> >& vkPath);
    int DijkstraSideCostPruneNew(int ID1, int ID2, vector<vector<pair<int, int> > >& adjSideDist, vector<vector<pair<int, int> > >& adjSideRoad, vector<int>& vSPTParent, vector<int>& vSPTParentEdge, vector<int>& vSPTDistance, vector<label>& vLabel, vector<int>& vIgnoredNode, vector<int>& vPath, vector<int>& vPathEdge, bool& bE, int& subLength,  int& dTmp);
    int eKSPNew(int ID1, int ID2, int k, vector<int>& kResults, vector<vector<int> >& vkPath, double t, int& countNumber, int& popPath, vector<float>& sim, float &SimTime);
    int eKSPInterval(int ID1, int ID2, int k, vector<int>& kResults, vector<vector<int> >& vkPath, double t, int& countNumber, int& popPath);
    int eKSPDiversity(int ID1, int ID2, int k, vector<int>& kResults, vector<vector<int> >& vkPath, double t, int& countNumber, int& popPath);
    int eKSPFCompare(int ID1, int ID2, int k, vector<int>& kResults, vector<vector<int> >& vkPath, double t, int& countNumber, int& popPath, vector<float>& sim);
    int eKSPCompare(int ID1, int ID2, int k, vector<int>& kResults, vector<vector<int> >& vkPath, double t, int& countNumber, int& popPath, float& percentage, float& SimTime, float& AveSim, float& minSim, float& maxSim);
    int eKSPPrune(int ID1, int ID2, int k, vector<int>& kResults, vector<vector<int> >& vkPath, double t, int& countNumber, int& popPath);
    int DynamicSimilarity(int ID1, int ID2, int k, vector<int>& kResults, vector<vector<int> >& vkPath, double t, int& countNumber, int& popPath, float& AveSim, float& minSim, float& maxSim);
    int EdgesBlocking(int ID1, int ID2, int k, vector<int>& kResults, vector<vector<int> >& vkPath, double t, int& countNumber, int& popPath);
    void FindRepeatedPath(vector<vector<int> >& vvPath);
};


#endif

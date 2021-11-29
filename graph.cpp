#include "graph.h"

void Graph::SPT(int root, vector<int>& vSPTDistance, vector<int>& vSPTParent,vector<int>& vSPTHeight, vector<int>& vSPTParentEdge, vector<vector<int> >& vSPT)
{
    benchmark::heap<2, int, int> queue(nodeNum);
    queue.update(root, 0);

    vector<bool> vbVisited(nodeNum, false);
    int topNodeID, neighborNodeID, neighborLength, neighborRoadID;
    vector<pair<int, int> >::iterator ivp;
    vSPTHeight[root] = 1;

    vSPTDistance[root] = 0;
    compareNode cnTop;
    while(!queue.empty())
    {
        int topDistance;
        queue.extract_min(topNodeID, topDistance);
        vbVisited[topNodeID] = true;
        for(int i = 0; i < (int)adjList[topNodeID].size(); i++)
        {
            neighborNodeID = adjList[topNodeID][i].first;
            neighborLength = adjList[topNodeID][i].second;
            neighborRoadID = adjListEdge[topNodeID][i].second;
            int d = vSPTDistance[topNodeID] + neighborLength;
            if(!vbVisited[neighborNodeID])
            {
                if(vSPTDistance[neighborNodeID] > d)
                {
                    vSPTDistance[neighborNodeID] = d;
                    queue.update(neighborNodeID, d);
                    vSPTParent[neighborNodeID] = topNodeID;
                    vSPTHeight[neighborNodeID] = vSPTHeight[topNodeID] + 1;
                    vSPTParentEdge[neighborNodeID] = neighborRoadID;
                }
            }
        }
    }
    //Construct SPT
    for(int i = 0; i < nodeNum; i++)
        if(vSPTParent[i] != -1)
            vSPT[vSPTParent[i]].push_back(i);
}

void Graph::rSPT(int root, vector<int>& vSPTDistance, vector<int>& vSPTParent, vector<int>& vSPTParentEdge, vector<vector<int> >& vSPT)
{
    benchmark::heap<2, int, int> queue(nodeNum);
    queue.update(root, 0);

    vector<bool> vbVisited(nodeNum, false);
    int topNodeID, neighborNodeID, neighborLength, neighborRoadID;
    vector<pair<int, int> >::iterator ivp;

    vSPTDistance[root] = 0;

    compareNode cnTop;
    while(!queue.empty())
    {
        int topDistance;
        queue.extract_min(topNodeID, topDistance);
        vbVisited[topNodeID] = true;
        for(int i = 0; i < (int)adjListR[topNodeID].size(); i++)
        {
            neighborNodeID = adjListR[topNodeID][i].first;
            neighborLength = adjListR[topNodeID][i].second;
            neighborRoadID = adjListEdgeR[topNodeID][i].second;
            int d = vSPTDistance[topNodeID] + neighborLength;
            if(!vbVisited[neighborNodeID])
            {
                if(vSPTDistance[neighborNodeID] > d)
                {
                    vSPTDistance[neighborNodeID] = d;
                    queue.update(neighborNodeID, d);
                    vSPTParent[neighborNodeID] = topNodeID;
                    vSPTParentEdge[neighborNodeID] = neighborRoadID;
                }
            }
        }
    }

    //Construct SPT
    for(int i = 0; i < nodeNum; i++)
        if(vSPTParent[i] != -1)
            vSPT[vSPTParent[i]].push_back(i);
}

vector<int> Graph::makeRMQDFS(int p, vector<vector<int> >& vSPT, vector<int>& vSPTParent){
    stack<int> sDFS;
    sDFS.push(p);
    vector<bool> vbVisited(nodeNum, false);
    while(!sDFS.empty())
    {
        int u = sDFS.top();
        if(vbVisited[u] == false)
        {
            int u = sDFS.top();
            EulerSeq.push_back(u);
            for(auto v = vSPT[u].end()-1; v != vSPT[u].begin()-1; v--)
                sDFS.push(*v);
            vbVisited[u] = true;
        }
        else
        {
            if(vSPTParent[u] != -1)
                EulerSeq.push_back(vSPTParent[u]);
            sDFS.pop();
        }
    }
    rEulerSeq.assign(EulerSeq.size(),-1);
    for(int i = 0; i < EulerSeq.size(); i++)
        rEulerSeq[EulerSeq[i]] = i;
    return EulerSeq;
}

vector<vector<int>> Graph::makeRMQ(int p, vector<vector<int> >& vSPT, vector<int>& vSPTHeight, vector<int>& vSPTParent){
    EulerSeq.clear();
    toRMQ.assign(nodeNum,0);
    RMQIndex.clear();
    std::chrono::high_resolution_clock::time_point t1;
    std::chrono::high_resolution_clock::time_point t2;
    std::chrono::duration<double> time_span;
    t1 = std::chrono::high_resolution_clock::now();
    makeRMQDFS(p, vSPT,vSPTParent);
    t2 = std::chrono::high_resolution_clock::now();
    time_span = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1);
    RMQIndex.push_back(EulerSeq);

    int m = EulerSeq.size();
    t1 = std::chrono::high_resolution_clock::now();
    for (int i = 2, k = 1; i < m; i = i * 2, k++){
        vector<int> tmp;
        tmp.assign(m,0);
        for (int j = 0; j < m - i; j++){
            int x = RMQIndex[k - 1][j], y = RMQIndex[k - 1][j + i / 2];
            if (vSPTHeight[x] < vSPTHeight[y])
                tmp[j] = x;
            else tmp[j] = y;
        }
        RMQIndex.push_back(tmp);
    }
    t2 = std::chrono::high_resolution_clock::now();
    time_span = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1);
    return RMQIndex;
}

int Graph::LCAQuery(int _p, int _q, vector<vector<int> >& vSPT, vector<int>& vSPTHeight, vector<int>& vSPTParent){
    int p = _p, q = _q;
    if (p > q){
        int x = p;
        p = q;
        q = x;
    }
    int len = q - p + 1;
    int i = 1, k = 0;
    while (i * 2 < len){
        i *= 2;
        k++;
    }
    q = q - i + 1;
    if (vSPTHeight[RMQIndex[k][p]] < vSPTHeight[RMQIndex[k][q]])
        return RMQIndex[k][p];
    else return RMQIndex[k][q];
}

int Graph::readUSMap(string filename)
{
    ifstream inGraph(filename);
    if(!inGraph)
        cout << "Cannot open Map " << filename << endl;
    cout << "Reading " << filename << endl;

    string line;
    do
    {
        getline(inGraph,line);
        if(line[0]=='p')
        {
            vector<string> vs;
            boost::split(vs,line,boost::is_any_of(" "),boost::token_compress_on);
            nodeNum = stoi(vs[2]);
            cout << "Nodenum " << nodeNum<<endl;
            edgeNum = 0;
        }
    }while(line[0]=='c'|| line[0]=='p');

    int ID1, ID2, length;
    adjList.resize(nodeNum);
    adjListR.resize(nodeNum);
    adjListEdge.resize(nodeNum);
    adjListEdgeR.resize(nodeNum);
    int edgeCount = 0;
    string a;
    while(!inGraph.eof())
    {
		vector<string> vs;
		boost::split(vs,line,boost::is_any_of(" "),boost::token_compress_on);
		ID1 = stoi(vs[1]) - 1;
		ID2 = stoi(vs[2]) - 1;
		length = stoi(vs[3]);

        struct Edge e;
        e.ID1 = ID1;
        e.ID2 = ID2;
        e.length = length;
        e.edgeID = edgeCount;

        bool bExisit = false;
        for(int i = 0; i < (int)adjList[ID1].size(); i++)
        {
            if(adjList[ID1][i].first == ID2)
            {
                bExisit = true;
                break;
            }
        }
        if(!bExisit)
        {
            vEdge.push_back(e);
            adjList[ID1].push_back(make_pair(ID2, length));
            adjListR[ID2].push_back(make_pair(ID1, length));
            adjListEdge[ID1].push_back(make_pair(ID2, edgeCount));
            adjListEdgeR[ID2].push_back(make_pair(ID1, edgeCount));
            edgeCount++;
        }
        getline(inGraph,line);
    }

    vbISO.assign(nodeNum, false);
    inGraph.close();

    return nodeNum;
}

vector<int> Graph::vDijkstra(int ID1)
{
    benchmark::heap<2, int, int> queue(adjList.size());
    queue.update(ID1, 0);

    vector<int> vDistance(nodeNum, INF);
    vector<bool> vbVisited(nodeNum, false);
    int topNodeID, neighborNodeID, neighborLength;
    vector<pair<int, int> >::iterator ivp;

    vDistance[ID1] = 0;
    countNum = 0;

    compareNode cnTop;
    while(!queue.empty())
    {
        int topDistance;
        queue.extract_min(topNodeID, topDistance);
        vbVisited[topNodeID] = true;
        countNum += 1;

        for(ivp = adjList[topNodeID].begin(); ivp != adjList[topNodeID].end(); ivp++)
        {
            neighborNodeID = (*ivp).first;
            neighborLength = (*ivp).second;
            int d = vDistance[topNodeID] + neighborLength;
            if(!vbVisited[neighborNodeID])
            {
                if(vDistance[neighborNodeID] > d)
                {
                    vDistance[neighborNodeID] = d;
                    queue.update(neighborNodeID, d);
                }
            }
        }
    }
    //cout << "Count: " << countNum << endl;
    return vDistance;
}
int Graph::Landmark(int ID1, int ID2)
{
    int d1, d2, est;
    int max = -1;
    for(int i = 0; i < 20; i++)
    {
        d1 = dNode[i][ID1];
        d2 = dNode[i][ID2];
        est = abs(d1 - d2);
        if(est > max)
            max = est;
    }
    return max;
}

int Graph::iBoundingAstar(int ID1, int ID2, unordered_set<int>& sRemovedNode, vector<int>& vPath, vector<int>& vPathEdge, int T)
{
    vector<int> vDistance(adjList.size(), INF);
    benchmark::heap<2, int, int> queue(adjList.size());
    vector<bool> vbVisited(adjList.size(), false);
    vector<int> vPrevious(adjList.size(), -1);
    vector<int> vPreviousEdge(adjList.size(), -1);
    int topNodeID, neighborNodeID, neighborLength, neighborRoadID;

    vDistance[ID1] = 0;
    queue.update(ID1, 0);

    while(!queue.empty())
    {
        int topDistance;
        queue.extract_min(topNodeID, topDistance);
        vbVisited[topNodeID] = true;
        if(topNodeID == ID2)
            break;
        for(int i = 0; i < (int)adjList[topNodeID].size(); i++)
        {
            neighborNodeID = adjList[topNodeID][i].first;
            if(sRemovedNode.find(neighborNodeID) != sRemovedNode.end())
                continue;
            neighborLength = adjList[topNodeID][i].second;
            neighborRoadID = adjListEdge[topNodeID][i].second;
            int d = vDistance[topNodeID] + neighborLength;
            if(!vbVisited[neighborNodeID])
            {
                if(vDistance[neighborNodeID] > d)
                {
                    vDistance[neighborNodeID] = d;
                    int lm = Landmark(adjList[topNodeID][i].first, ID2);
                    if( d + lm <= T)
                    {
                        queue.update(neighborNodeID, d + lm);
                        vPrevious[neighborNodeID] = topNodeID;
                        vPreviousEdge[neighborNodeID] = neighborRoadID;
                    }
                }
            }
        }
    }

    vPath.clear();
    vPathEdge.clear();
    vPath.push_back(ID2);
    int p = vPrevious[ID2];
    int e = vPreviousEdge[ID2];
    while(p != -1)
    {
        vPath.push_back(p);
        vPathEdge.push_back(e);
        e = vPreviousEdge[p];
        p = vPrevious[p];
    }
    reverse(vPath.begin(), vPath.end());
    reverse(vPathEdge.begin(), vPathEdge.end());

    return vDistance[ID2];
}


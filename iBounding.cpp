#include "graph.h"

int Graph::iBounding(int ID1, int ID2, int k, vector<int>& kResults)
{
    vector<int> vPath, vPathEdge;
    vector<vector<int> > vvResult;
    vvResult.reserve(k);

    vector<int> vDistance;
    vector<int> Path;
    vector<vector<int> > vvPathCandidate;	//nodes
    vector<vector<int> > vvPathCandidateEdge;//edges
    vector<int> vPathParent;				//Deviated from
    vector<int> vPathParentPos;				//Deviated Pos from Parent
    vector<vector<int> > vvPathRemovedEdge; //Removed Edges when computed
    vector<unordered_set<int> > vvPathRemovedNode; //Removed Nodes when computed

    benchmark::pHeap<2, int, int, int> lPath(adjList.size());

    int d = DijkstraPath(ID1, ID2, vPath, vPathEdge);
    int lb;
    int pWeight = d;
    vDistance.push_back(d);
    vvPathCandidate.push_back(vPath);
    vvPathCandidateEdge.push_back(vPathEdge);
    vPathParent.push_back(-1);
    vPathParentPos.push_back(0);
    vector<int> vTmp;
    unordered_set<int>  sB;
    vvPathRemovedEdge.push_back(vTmp);
    vvPathRemovedNode.push_back(sB);

    lPath.update(vvPathCandidate.size()-1, pWeight, pWeight);

    int topSpaceID;
    dNode.clear();
    for(int i = 0; i < 20; i++)
    {
        Node[i] = rand() % nodeNum;
        dNode.push_back(vDijkstra(Node[i]));
    }

    while((int)vvResult.size() < k)
    {
        lPath.extract_min(topSpaceID,lb,pWeight);
        //cout << topSpaceID << "......" << lb << "......" << pWeight << endl;
        vector<int> vRemovedEdgeTmp;
        unordered_set<int> sRemovedNodeTmp;
        if(pWeight != 0){
            kResults.push_back(pWeight);
            vvResult.push_back(vvPathCandidate[topSpaceID]);
            /*for(auto ie = vvPathCandidateEdge[topSpaceID].begin(); ie != vvPathCandidateEdge[topSpaceID].end(); ie++)
            {
                int eID1 = vEdge[*ie].ID1;
                eID2 = vEdge[*ie].ID2;
                cout << eID1 << "\t";
            }
            cout << eID2 << "\t";
            cout << endl;*/
            for(auto ie = vvPathRemovedEdge[topSpaceID].begin(); ie != vvPathRemovedEdge[topSpaceID].end(); ie++)
            {
                int eID1 = vEdge[*ie].ID1;
                int eID2 = vEdge[*ie].ID2;
                //cout << eID1 << "," << eID2 << endl;
                sRemovedNodeTmp.insert(eID1);
                for(int i = 0; i < (int)adjList[eID1].size(); i++)
                {
                    if(adjList[eID1][i].first == eID2)
                    {
                        adjList[eID1].erase(adjList[eID1].begin() + i);
                        adjListEdge[eID1].erase(adjListEdge[eID1].begin()+i);
                    }
                }
                vRemovedEdgeTmp.push_back(*ie);
            }
            //sub-path
            vector<int> vPreviousNode;
            vector<int> vPreviousEdge;
            int subLength = 0;
            int i;
            //cout << vPathParentPos[topSpaceID] << endl;
            for(i = 0; i < vPathParentPos[topSpaceID]; i++)
            {
                vPreviousNode.push_back(vvPathCandidate[topSpaceID][i]);
                vPreviousEdge.push_back(vvPathCandidateEdge[topSpaceID][i]);
                subLength += vEdge[vvPathCandidateEdge[topSpaceID][i]].length;
            }
            //Remove the edges incrementally for each new path
            for(auto ie = vvPathCandidateEdge[topSpaceID].begin() + i; ie != vvPathCandidateEdge[topSpaceID].end(); ie++)
            {
                int eID1 = vEdge[*ie].ID1;
                int eID2 = vEdge[*ie].ID2;
                //cout << eID1 << "," << eID2 << endl;
                for(int j = 0; j < (int)adjList[eID1].size(); j++)
                {
                    if(adjList[eID1][j].first == eID2)
                    {
                        adjList[eID1].erase(adjList[eID1].begin() + j);
                        adjListEdge[eID1].erase(adjListEdge[eID1].begin()+j);
                    }
                }
                vRemovedEdgeTmp.push_back(*ie);
                sRemovedNodeTmp.insert(eID1);

                subLength += vEdge[*ie].length;


                vector<int> vPreviousNodeTmp = vPreviousNode;
                vector<int> vPreviousEdgeTmp = vPreviousEdge;
                vPreviousEdge.push_back(*ie);
                vPreviousNode.push_back(eID2);

                vDistance.push_back(subLength);
                vvPathCandidate.push_back(vPreviousNodeTmp);
                vvPathCandidateEdge.push_back(vPreviousEdgeTmp);
                vPathParent.push_back(topSpaceID);
                vPathParentPos.push_back(ie - vvPathCandidateEdge[topSpaceID].begin());
                vvPathRemovedEdge.push_back(vRemovedEdgeTmp);
                vvPathRemovedNode.push_back(sRemovedNodeTmp);
                //cout << vvPathCandidate[1].size() << endl;
                int vlb = INF;
                for(int i = 0; i < (int)adjList[eID1].size(); i++)
                {
                    if(adjList[eID1][i].first != eID2)
                    {
                        int w = subLength - vEdge[*ie].length + adjList[eID1][i].second;
                        int lm = Landmark(adjList[eID1][i].first, ID2);
                        if(w+lm < vlb)
                        {
                            vlb = w+lm;
                        }
                    }
                }
                //lb = pWeight;
                lb = max(pWeight,vlb);
                //cout << vvPathCandidate.size()-1 << ",,,,," << lb << endl;
                lPath.update(vvPathCandidate.size()-1, lb, 0);
            }
            //Recover the removed edges
            for(auto ie = vRemovedEdgeTmp.begin(); ie != vRemovedEdgeTmp.end(); ie++)
            {
                int eID1 = vEdge[*ie].ID1;
                int eID2 = vEdge[*ie].ID2;
                adjList[eID1].push_back(make_pair(eID2, vEdge[*ie].length));
                adjListEdge[eID1].push_back(make_pair(eID2, *ie));
            }
        }
        else
        {
            int T = 1.1 * lb;
            int eID1;
            int subLength = 0;
            auto m = vvPathCandidateEdge[topSpaceID].end() - 1;
            //cout << topSpaceID << "...." << vvPathCandidateEdge[topSpaceID].size() << endl;
            if (vvPathCandidateEdge[topSpaceID].size() == 0)
            {
                eID1 = ID1;
            }
            else
            {
                eID1 = vEdge[*m].ID2;
            }
            vector<int> vRemovedEdgeTmp;
            unordered_set<int> sRemovedNodeTmp;
            for(auto ie = vvPathRemovedEdge[topSpaceID].begin(); ie != vvPathRemovedEdge[topSpaceID].end(); ie++)
            {
                int eID1 = vEdge[*ie].ID1;
                int eID2 = vEdge[*ie].ID2;
                //cout << eID1 << "," << eID2 << endl;
                sRemovedNodeTmp.insert(eID1);
                for(int i = 0; i < (int)adjList[eID1].size(); i++)
                {
                    if(adjList[eID1][i].first == eID2)
                    {
                        adjList[eID1].erase(adjList[eID1].begin() + i);
                        adjListEdge[eID1].erase(adjListEdge[eID1].begin()+i);
                    }
                }
                vRemovedEdgeTmp.push_back(*ie);
            }
            int n;
            for(n = 0; n < vPathParentPos[topSpaceID]; n++)
            {
                subLength += vEdge[vvPathCandidateEdge[topSpaceID][n]].length;
            }
            //cout << topSpaceID << "...." << eID1 << "...." << ID2 << "..." << vDistance[ID2]+subLength << endl;
            int dTmp = iBoundingAstar(eID1, ID2, sRemovedNodeTmp, vPath, vPathEdge, T) + subLength;
            vDistance[topSpaceID] = dTmp;
            vvPathCandidate[topSpaceID].insert(vvPathCandidate[topSpaceID].end(), vPath.begin() + 1, vPath.end());
            vvPathCandidateEdge[topSpaceID].insert(vvPathCandidateEdge[topSpaceID].end(), vPathEdge.begin(), vPathEdge.end());
            if(dTmp > 0)
            {
                lPath.update(topSpaceID, dTmp, dTmp);
            }
            else
            {
                lPath.update(topSpaceID, T, 0);
            }
            for(auto ie = vRemovedEdgeTmp.begin(); ie != vRemovedEdgeTmp.end(); ie++)
            {
                int eID1 = vEdge[*ie].ID1;
                int eID2 = vEdge[*ie].ID2;
                adjList[eID1].push_back(make_pair(eID2, vEdge[*ie].length));
                adjListEdge[eID1].push_back(make_pair(eID2, *ie));
            }
        }
    }
    return vvResult.size();
}

#include "graph.h"

int Graph::cTKSPD(int ID1, int ID2, int k, vector<int>& kResults,double t,int& countNumber, int& popPath) {
    countNumber = 0;
    popPath = 0;
    vector<int> vSPTDistance(nodeNum, INF);
    vector<int> vSPTParent(nodeNum, -1);
    vector<int> vSPTParentEdge(nodeNum, -1);
    vector<int> vTmp;
    vector<vector<int> > vSPT(nodeNum, vTmp);
    vector<int> vSPTCost(nodeNum, INF);
    vector<int> vSPTCostLB(nodeNum, INF);
    vector<int> vCostUB(nodeNum, INF);
    rSPT(ID2, vSPTDistance, vSPTParent, vSPTParentEdge, vSPT);
    dNode.clear();
    for(int i = 0; i < 20; i++)
    {
        Node[i] = rand() % nodeNum;
        dNode.push_back(vDijkstra(Node[i]));
    }

    vector<int> vPath, vPathEdge;
    vector<vector<int> > vvResult;
    vvResult.reserve(k);
    vector<unordered_set<int> > pResult;
    unordered_set<int> pa;
    pResult.reserve(k);

    vector<int> vDistance;
    vector<int> Path;
    vector<vector<int> > vvPathCandidate;    //nodes
    vector<vector<int> > vvPathCandidateEdge;//edges
    vector<int> vPathParent;                //Deviated from
    vector<int> vPathParentPos;                //Deviated Pos from Parent
    vector<vector<int> > vvPathRemovedEdge; //Removed Edges when computed
    vector<unordered_set<int> > vvPathRemovedNode; //Removed Nodes when computed
    float sim;

    benchmark::pHeap<2, int, int, int> lPath(nodeNum);

    int p = ID1;
    vPath.push_back(p);
    int d = 0;
    while(vSPTParent[p] != -1)
    {
        vPathEdge.push_back(vSPTParentEdge[p]);
        d += vEdge[vSPTParentEdge[p]].length;
        p = vSPTParent[p];
        vPath.push_back(p);
    }

    int lb;
    int pWeight = d;
    vDistance.push_back(d);
    vvPathCandidate.push_back(vPath);
    vvPathCandidateEdge.push_back(vPathEdge);
    vPathParent.push_back(-1);
    vPathParentPos.push_back(0);
    vector<int> Tmp;
    unordered_set<int> sB;
    vvPathRemovedEdge.push_back(Tmp);
    vvPathRemovedNode.push_back(sB);

    lPath.update(vvPathCandidate.size() - 1, pWeight, pWeight);

    int topSpaceID;
    dNode.clear();
    for(int i = 0; i < 20; i++)
    {
        Node[i] = rand() % nodeNum;
        dNode.push_back(vDijkstra(Node[i]));
    }
    while ((int) vvResult.size() < k && !lPath.empty()) {
        //cout << vvResult.size() << endl;
        lPath.extract_min(topSpaceID, lb, pWeight);
        //cout << topSpaceID << "......" << lb << "......" << pWeight << endl;
        vector<int> vRemovedEdgeTmp;
        unordered_set<int> sRemovedNodeTmp;
        if (pWeight != 0) {
            popPath++;
            /*kResults.push_back(pWeight);
            vvResult.push_back(vvPathCandidate[topSpaceID]);*/
            float addLength = 0;
            if (vvResult.size() == 0)
            {
                //cout << topSpaceID << "......" << lb << "......" << pWeight << endl;
                kResults.push_back(pWeight);
                vvResult.push_back(vvPathCandidateEdge[topSpaceID]);
                for(auto ie = vvPathCandidateEdge[topSpaceID].begin(); ie != vvPathCandidateEdge[topSpaceID].end(); ie++)
                {
                    pa.insert(*ie);
                }
                pResult.push_back(pa);
                pa.clear();
            }
            else
            {
                //cout << topSpaceID << "......" << lb << "......" << pWeight << endl;
                for(int i = 0; i < pResult.size(); i++)
                {
                    //cout << i << "..." << pResult.size() << "..." << topSpaceID << "....." << preAddLength <<endl;
                    for(auto ie = vvPathCandidateEdge[topSpaceID].begin(); ie != vvPathCandidateEdge[topSpaceID].end(); ie++)
                    {
                        if (pResult[i].find(*ie) != pResult[i].end())
                            addLength += vEdge[*ie].length;
                    }
					//Sim1
                    sim = addLength/(kResults[i]+pWeight-addLength);

					/*//Sim 2 
					sim = addLength / (2*kResults[i]) + addLength / (2*pWeight);

					//Sim 3
					sim = sqrt((addLength*addLength) / ((double)kResults[i]*(double)pWeight));	

					//Sim 4
					int maxLength;
					if(addLength > pWeight)
						maxLength = addLength;
					else
						maxLength = pWeight;
					sim = addLength / maxLength;
					
					//Sim5
					sim = addLength / kResults[i];*/




					//sim = (addLength + preAddLength)/(kResults[i]+pWeight-addLength-preAddLength);
                    //cout << topSpaceID << "....." << sim << "...." << pWeight <<endl;
                    addLength = 0;
                    if (sim > t)
                        break;
                }
                //cout << sim << endl;
                if (sim <= t)
                {
                    cout << sim << endl;
                    kResults.push_back(pWeight);
                    vvResult.push_back(vvPathCandidateEdge[topSpaceID]);
                    pa.clear();
                    for(auto ie = vvPathCandidateEdge[topSpaceID].begin(); ie != vvPathCandidateEdge[topSpaceID].end(); ie++)
                    {
                        pa.insert(*ie);
                    }
                    pResult.push_back(pa);
                }
            }
            for (auto ie = vvPathRemovedEdge[topSpaceID].begin(); ie != vvPathRemovedEdge[topSpaceID].end(); ie++) {
                int eID1 = vEdge[*ie].ID1;
                int eID2 = vEdge[*ie].ID2;
                //cout << eID1 << "," << eID2 << endl;
                sRemovedNodeTmp.insert(eID1);
                for (int i = 0; i < (int) adjList[eID1].size(); i++) {
                    if (adjList[eID1][i].first == eID2)
                    {
                        adjList[eID1].erase(adjList[eID1].begin() + i);
                        adjListEdge[eID1].erase(adjListEdge[eID1].begin() + i);
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
            for (i = 0; i < vPathParentPos[topSpaceID]; i++) {
                vPreviousNode.push_back(vvPathCandidate[topSpaceID][i]);
                vPreviousEdge.push_back(vvPathCandidateEdge[topSpaceID][i]);
                subLength += vEdge[vvPathCandidateEdge[topSpaceID][i]].length;
            }
            //Remove the edges incrementally for each new path
            for (auto ie = vvPathCandidateEdge[topSpaceID].begin() + i; ie != vvPathCandidateEdge[topSpaceID].end(); ie++) {
                int eID1 = vEdge[*ie].ID1;
                int eID2 = vEdge[*ie].ID2;
                //cout << eID1 << "," << eID2 << endl;
                for (int j = 0; j < (int) adjList[eID1].size(); j++)
                {
                    if (adjList[eID1][j].first == eID2)
                    {
                        adjList[eID1].erase(adjList[eID1].begin() + j);
                        adjListEdge[eID1].erase(adjListEdge[eID1].begin() + j);
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
                for (int i = 0; i < (int) adjList[eID1].size(); i++)
                {
                    if (adjList[eID1][i].first != eID2)
                    {
                        int w = subLength - vEdge[*ie].length + adjList[eID1][i].second;
                        int lm = vSPTDistance[adjList[eID1][i].first];
                        if (w + lm < vlb)
                        {
                            vlb = w + lm;
                        }
                    }
                }
                //lb = pWeight;
                lb = max(pWeight, vlb);
                //cout << vvPathCandidate.size()-1 << "...." << lb << endl;
                lPath.update(vvPathCandidate.size() - 1, lb, 0);
            }
            //Recover the removed edges
            for (auto ie = vRemovedEdgeTmp.begin(); ie != vRemovedEdgeTmp.end(); ie++) {
                int eID1 = vEdge[*ie].ID1;
                int eID2 = vEdge[*ie].ID2;
                adjList[eID1].push_back(make_pair(eID2, vEdge[*ie].length));
                adjListEdge[eID1].push_back(make_pair(eID2, *ie));
            }
        }
        else
        {
            vector<double> lbp2(vvResult.size());
            int addLength = 0;
            vector<int> overLength(vvResult.size(), INF);
            for(int i = 0; i < pResult.size(); i++)
            {
                for(auto ie = vvPathCandidateEdge[topSpaceID].begin(); ie != vvPathCandidateEdge[topSpaceID].end(); ie++)
                {
                    if (pResult[i].find(*ie) != pResult[i].end())
                    {
                        addLength += vEdge[*ie].length;
                    }
                }


				//Sim 1
                lbp2[i] = addLength * (1 + 1/t) - kResults[i];

				//Sim2
				/*if(2*t*kResults[i] > addLength)
					lbp2[i] = ((double)addLength * (double)kResults[i]) / (2*t*kResults[i] - addLength);
				else
					lbp2[i] = INF;*/

				//Sim3
				//lbp2[i] = ((double)addLength * (double)addLength) / t*t*(double)kResults[i];

				//Sim4	
				/*if(vDistance[topSpaceID] >= kResults[i])
					lbp2[i] = addLength /t;
				else
					lbp2[i] = kResults[i];*/

				//Sim5
				/*if(addLength >= t*kResults[i])
					lbp2[i] = INF;
				else
					lbp2[i] = kResults[i];*/
                addLength = 0;
            }
            int lb2 = lbp2[0];
            for (int n = 0; n != lbp2.size(); n++)
            {
                if (lb2 < lbp2[n])
                {
                    lb2 = lbp2[n];
                }
            }
            if (lb2 > lb)
            {
                lPath.update(topSpaceID, lb2, 0);
				//cout << topSpaceID << " lb2: "<< lb2 << " lb: "<< lb<< endl;
            }
            else
            {
                countNumber += 1;
                int T = 1.1 * lb;
                int eID1;
                auto m = vvPathCandidateEdge[topSpaceID].end() - 1;

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
                int subLength = 0;
                for (int n = 0; n < vPathParentPos[topSpaceID]; n++)
                {
                    subLength += vEdge[vvPathCandidateEdge[topSpaceID][n]].length;
                }
                for (auto ie = vvPathRemovedEdge[topSpaceID].begin(); ie != vvPathRemovedEdge[topSpaceID].end(); ie++)
                {
                    int mID1 = vEdge[*ie].ID1;
                    int mID2 = vEdge[*ie].ID2;
                    //cout << mID1 << "," << mID2 << endl;
                    sRemovedNodeTmp.insert(mID1);
                    for (int i = 0; i < (int) adjList[mID1].size(); i++)
                    {
                        if (adjList[mID1][i].first == mID2)
                        {
                            adjList[mID1].erase(adjList[mID1].begin() + i);
                            adjListEdge[mID1].erase(adjListEdge[mID1].begin() + i);
                        }
                    }
                    vRemovedEdgeTmp.push_back(*ie);
                }
                int dTmp = iBoundingAstar(eID1, ID2, sRemovedNodeTmp, vPath, vPathEdge, T) + subLength;
                //int dTmp = DijkstraPath2(eID1, ID2, sRemovedNodeTmp, vPath, vPathEdge) + subLength;
				vDistance[topSpaceID] = dTmp;
                vvPathCandidate[topSpaceID].insert(vvPathCandidate[topSpaceID].end(), vPath.begin() + 1, vPath.end());
                vvPathCandidateEdge[topSpaceID].insert(vvPathCandidateEdge[topSpaceID].end(), vPathEdge.begin(), vPathEdge.end());
				//lPath.update(topSpaceID, dTmp, dTmp);
                if(dTmp > 0)
                {
                    lPath.update(topSpaceID, dTmp, dTmp);
                }
                else
                {
                    lPath.update(topSpaceID, T, 0);
                }
                for (auto ie = vRemovedEdgeTmp.begin(); ie != vRemovedEdgeTmp.end(); ie++) {
                    int eID1 = vEdge[*ie].ID1;
                    int eID2 = vEdge[*ie].ID2;
                    adjList[eID1].push_back(make_pair(eID2, vEdge[*ie].length));
                    adjListEdge[eID1].push_back(make_pair(eID2, *ie));
                }
            }
        }
    }
    cout << "cTKSPD(iB No Sim) countNumber: "<< countNumber << " Pop Path Length: " << popPath << endl;
    return vvResult.size();
}


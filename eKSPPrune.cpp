#include "graph.h"

int Graph::eKSPPrune(int ID1, int ID2, int k, vector<int>& kResults, vector<vector<int> >& vkPath, double t, int& countNumber, int& popPath)
{
    //Shortest Path Tree Info
    countNumber = 0;
    popPath = 0;
    vector<int> vSPTDistance(nodeNum, INF);
    vector<int> vSPTParent(nodeNum, -1);
    vector<int> vSPTHeight(nodeNum, -1);
    vector<int> vSPTParentEdge(nodeNum, -1);
    vector<int> vSPTChildren(nodeNum, -1);
    vector<int> vTmp;
    vector<vector<int>> vSPT(nodeNum, vTmp); //Tree from root
    SPT(ID1, ID2, vSPTDistance, vSPTParent, vSPTHeight, vSPTParentEdge, vSPT);

	bool bCountNumber = true;
	vector<vector<int> > vkPath2;
	vkPath = vkPath2;
	vector<int> kResults2;
	kResults = kResults2;

    //LCA
    vector<vector<float> > vPathLCA;
    vector<vector<int> > RMQ = makeRMQ(ID1, vSPT, vSPTHeight,vSPTParent);
    vector<int> vEdgeReducedLength(vEdge.size(), INF);
    for(int i = 0; i < (int)vEdge.size(); i++)
        vEdgeReducedLength[i] = vEdge[i].length + vSPTDistance[vEdge[i].ID1] - vSPTDistance[vEdge[i].ID2];

    vector<vector<int> > vvResult;	//Exact Path
    vvResult.reserve(k);
    vector<int> vDistance;
    vector<vector<int>> vAncestor; // Ancestors in the result path set
    vector<int> vPathParent;	//Deviated from
    vector<int> vPathParentPos;	//Deviated Pos from Parent
    vector<int> vPathDeviation; //deviation node
    vector<vector<int> > vPathLCANode; //LCA Node with path 1
    vector<int> vPathDevPrime; // The other node of the deviation edge
    vector<vector<float> > vPathFix;
    vector<float> mPathFix;
	//Store addLength
	vector<vector<float> > vPathAddLength;
    vector<int> dEdge;
    vector<bool> bPath;
    vector<vector<int> > vvPathCandidate;	 //nodes
    vector<vector<int> > vvPathCandidateEdge;//edges
    vector<unordered_map<int, int> > vmPathNodePos;	//Position of the non-fixed vertices
    //The size of mvPathNodePos indicates the fixed pos
    vector<vector<pair<int, int> > > vvPathNonTree;
    vector<multimap<int, int> > vmArc;
    vector<int> vFather;
    vFather.push_back(-1);
	vector<unordered_set<int> > pResult;

    float sim;

    vector<int> vPath;
    vector<int> vPathEdge;
    vPath.push_back(ID2);
    vector<pair<int, int> > vPathNonTree;
    int p = vSPTParent[ID2];
    int e = vSPTParentEdge[ID2];
    multimap<int, int> mArc;
    float sim1;
    float addLength1 = 0;
    int oldP = ID2;

    while(p != -1)
    {
        if(sim1 <= t)
        {
            for(int i = 0; i < (int)adjListEdgeR[oldP].size(); i++)
            {
                int eID = adjListEdgeR[oldP][i].second;
                if(eID != e)
                    mArc.insert(make_pair(vEdgeReducedLength[eID], eID));
            }
        }
		addLength1 += vEdge[e].length;
        sim1 = addLength1 / vSPTDistance[ID2];
        vPath.push_back(p);
        oldP = p;
        vPathEdge.push_back(e);
        e = vSPTParentEdge[p];
        p = vSPTParent[p];
    }
    vmArc.push_back(mArc);

    reverse(vPath.begin(), vPath.end());
    reverse(vPathEdge.begin(), vPathEdge.end());
    unordered_map<int, int> mPos;
    for(int i = 0; i < (int)vPath.size(); i++)
        mPos[vPath[i]] = i;
    vmPathNodePos.push_back(mPos);

    benchmark::heap<2, int, int> qPath(nodeNum);
    vvPathCandidate.push_back(vPath);
    vvPathCandidateEdge.push_back(vPathEdge);
    vDistance.push_back(vSPTDistance[ID2]);
    bPath.push_back(true);
    dEdge.push_back(0);
    vPathParent.push_back(-1);
    vPathParentPos.push_back(0);
    vPathDeviation.push_back(ID2);
    vector<int> vTmpLCANode;
    vTmpLCANode.push_back(ID2);
    vPathLCANode.push_back(vTmpLCANode);
    vPathDevPrime.push_back(ID2);
    vector<float> vTmpFix;
    vTmpFix.push_back(0);
    vPathFix.push_back(vTmpFix);

	vector<float> vTmpAddLength;
	vTmpAddLength.push_back(0);
	vPathAddLength.push_back(vTmpAddLength);

    vector<int> vTmpAncestor;
    vTmpAncestor.push_back(-1);
    vAncestor.push_back(vTmpAncestor);
    vector<float> vTmpLCA;
    vTmpLCA.push_back(vSPTDistance[ID2]);
    vPathLCA.push_back(vTmpLCA);
    qPath.update(vvPathCandidate.size()-1, vSPTDistance[ID2]);

    vector<int> vResultID; 
	vector<double> vResultThreshold;
    int topPathID, topPathDistance;
    int pcount = 0;
    int oldDistance = -1;
    bool bError = false;
    while((int)kResults.size() < k && !qPath.empty())
    {
        float addLength = 0;
        pcount++;
        qPath.extract_min(topPathID, topPathDistance);
        if(topPathDistance < oldDistance)
            cout<< "Error" <<endl;
        oldDistance = topPathDistance;
        //Loop Test
        unordered_set<int> us;
        bool bTopLoop = false;
        for(auto& v : vvPathCandidate[topPathID])
        {
            if(us.find(v) == us.end())
                us.insert(v);
            else
            {
                bTopLoop = true;
                break;
            }
        }
        if(!bTopLoop)
        {
            popPath++;
            int n = 0;
            if (vvResult.size() == 0)
            {
                vvResult.push_back(vvPathCandidateEdge[topPathID]);
                kResults.push_back(topPathDistance);
                vkPath.push_back(vvPathCandidate[topPathID]);
                vResultID.push_back(topPathID);
                mPathFix.push_back(0);
				unordered_set<int> pTmp;
				for(auto ie = vvPathCandidateEdge[topPathID].begin(); ie != vvPathCandidateEdge[topPathID].end(); ie++)
				{
					pTmp.insert(*ie);
				}
				pResult.push_back(pTmp); 
				vResultThreshold.push_back(vDistance[topPathID] * t);
			}
			
            else{
                for (int i = 0; i < vResultID.size(); i++) {
                    bool vFind = false;
                    for(int j = 0; j < vAncestor[topPathID].size(); j++)
                    {
                        if(vResultID[i] == vAncestor[topPathID][j])
                        {
                            vFind = true;
                            float simAdd = vPathFix[topPathID][j] + vPathLCA[topPathID][j] + mPathFix[i];
                            sim = simAdd / kResults[i];
                        }
                    }
                    if(vFind == false)
                    {
                        for (auto ie = vvPathCandidateEdge[topPathID].begin(); ie != vvPathCandidateEdge[topPathID].end(); ie++) {
                            if(pResult[i].find(*ie) != pResult[i].end())
                                addLength += vEdge[*ie].length;
                        }
                        //sim = addLength / kResults[i];
                        //addLength = 0;
                    
						//Sim 1
						//sim = addLength / (kResults[i] + topPathDistance - addLength);
						//Sim 2 
						//sim = addLength / (2*kResults[i]) + addLength / (2*topPathDistance);

						//Sim 3
						//sim = sqrt((addLength*addLength) / ((double)kResults[i]*(double)topPathDistance));
							
						//Sim 4
						/*int maxLength;
						if(addLength > topPathDistance)
							maxLength = addLength;
						else
							maxLength = topPathDistance;
						sim = addLength / maxLength;*/

						//Sim 5
						sim = addLength / kResults[i];
						addLength = 0;
					}
                    if (sim > t)
                        break;
                }
                if (sim <= t) {
                    cout << sim << " topPathID:" << topPathID << endl;
                    kResults.push_back(topPathDistance);
                    vvResult.push_back(vvPathCandidateEdge[topPathID]);
                    vkPath.push_back(vvPathCandidate[topPathID]);
                    vResultID.push_back(topPathID);
                    bPath[topPathID] = true;
                    //all path deviate from this path will contain the mFix part
                    mPathFix.push_back(vDistance[vFather[topPathID]] - vSPTDistance[vPathDeviation[topPathID]] + dEdge[topPathID]);
					unordered_set<int> pTmp2;
					for(auto ie = vvPathCandidateEdge[topPathID].begin(); ie != vvPathCandidateEdge[topPathID].end(); ie++)
					{
						pTmp2.insert(*ie);
					}
					pResult.push_back(pTmp2);
					vResultThreshold.push_back(vDistance[topPathID] * t);
                }
            }
        }
        vector<int> vTwo;
        vTwo.push_back(topPathID);
        if(vFather[topPathID] != -1 &&  !vmArc[vFather[topPathID]].empty())
            vTwo.push_back(vFather[topPathID]);
        for(auto& pID : vTwo)
        {
			bool bFixSim = true;
            bool bLoop = true;
			//bool bSim = true;
            while(bLoop)
            {
                //No More Candidate from current class
                if(vmArc[pID].empty())
                {
                    vvPathCandidate[pID].clear();
                    vvPathCandidateEdge[pID].clear();
                    vmPathNodePos[pID].clear();
                    break;
                }
                int mineID;
                int eReducedLen;
                auto it = vmArc[pID].begin();
                eReducedLen = (*it).first;
                mineID = (*it).second;
                countNumber += 1;
                vmArc[pID].erase(it);

                //eNodeID1 is deviation prime node
                int eNodeID1 = vEdge[mineID].ID1;
                int eNodeID2 = vEdge[mineID].ID2;

                //loop test
                bool bFixedLoop = false;
                unordered_set<int> sE;
                for(int i = vmPathNodePos[pID][eNodeID2]; i < (int)vvPathCandidate[pID].size(); i++)
                {
                    if(sE.find(vvPathCandidate[pID][i]) == sE.end())
                        sE.insert(vvPathCandidate[pID][i]);
                    else
                    {
                        bFixedLoop = true;
                        break;
                    }
                }
                if(bFixedLoop)
                    continue;

                int distTmp = vDistance[pID] - vSPTDistance[eNodeID2] + vEdge[mineID].length;
                bLoop = false;
                vPathDevPrime.push_back(eNodeID1);
                vPathDeviation.push_back(eNodeID2);
                multimap<int, int> mArcTmp;
				//vector<int, int> mArcTmpSim;
                vPath.clear();
                vPathEdge.clear();
                int p = eNodeID1;
                int e = vSPTParentEdge[p];
                int dist = vDistance[pID] - vSPTDistance[eNodeID2] + vSPTDistance[eNodeID1] + vEdge[mineID].length;
                vDistance.push_back(dist);

				vector<float> addLength2;
				vector<float> sim2;
				for(int i = 0; i < vvResult.size(); i++){
					addLength2.push_back(0);
					sim2.push_back(0);
				}

				bool bFixSim = true;
				for(int j = vmPathNodePos[pID][eNodeID2]; j < vvPathCandidateEdge[pID].size(); j++)
				{
					for(int i = 0; i < vvResult.size(); i++)
					{
						if(pResult[i].find(vvPathCandidateEdge[pID][j]) != pResult[i].end())
								addLength2[i] += vEdge[vvPathCandidateEdge[pID][j]].length;
						if(addLength2[i] > vResultThreshold[i])
						{
							bFixSim = false;
							break;
						}
					}
					if(!bFixSim)
						break;
				}
				bool bSim = true;

                while(p != -1 && bFixSim)
				{
					sE.insert(p);
                    if(bSim)
					{
                        for(int i = 0; i < (int)adjListEdgeR[p].size(); i++)
                        {
                            int reID = adjListEdgeR[p][i].second;
                            int eID1 = vEdge[reID].ID1;

                            if(sE.find(eID1) != sE.end())
                                continue;

                            if(reID != e && reID != mineID)
							{
								bool breID = true;
								for(int j = 0; j < vvResult.size(); j++)
								{
									if(pResult[j].find(reID) != pResult[j].end())
									{
										if((addLength2[i] + vEdge[reID].length) > vResultThreshold[i])
											breID = false;
										break;
									}
									if(!breID)
										break;
								}

								if(breID)
									mArcTmp.insert(make_pair(vEdgeReducedLength[reID], reID));
									
							}
                        }
						
						for(int i = 0; i < vvResult.size(); i++)
						{
							int count = 0;
							if(pResult[i].find(e) != pResult[i].end())
							{
								addLength2[i] += vEdge[e].length; 
							}
							if(addLength2[i] > vResultThreshold[i]) 
							{
								bSim = false;
								break;
							}
						}
                    }

					if(bSim)
					{
						vPath.push_back(p);
						vPathEdge.push_back(e);
					    p = vSPTParent[p];
						if(vSPTParent[p] == -1)
					    {
							vPath.push_back(p);
							break;
						}
						e = vSPTParentEdge[p];
					}
					else
					{
						vPath.push_back(p);
						vPathEdge.push_back(e);
					    p = vSPTParent[p];
						vPath.push_back(p);
						break;
					}
				}
                dEdge.push_back(vEdge[mineID].length);
                if (bPath[pID])
                {
                    if(pID == 0)
                    {
                        vector<int> tmpAnc;
                        tmpAnc.push_back(pID);
                        vAncestor.push_back(tmpAnc);
                    }
                    else{
                        vAncestor.push_back(vAncestor[pID]);
                        vAncestor[vDistance.size()-1].push_back(pID);
                    }
                }
                else{
                    vAncestor.push_back(vAncestor[pID]);
                }

                for(int i = 0; i < vAncestor[vDistance.size()-1].size(); i++)
                {
                    // find LCA
                    int _p, _q;
                    _p = rEulerSeq[vPathDevPrime[vAncestor[vDistance.size()-1][i]]];
                    _q = rEulerSeq[eNodeID1];
                    int LCA = LCAQuery(_p, _q, vSPT,  vSPTHeight,vSPTParent);
                    int dLCA = vSPTDistance[LCA];
                    if(i == 0)
                    {
                        vector<float> mTmpLCA;
                        vector<int> mTmpLCANode;
                        mTmpLCA.push_back(dLCA);
                        vPathLCA.push_back(mTmpLCA);
                        mTmpLCANode.push_back(LCA);
                        vPathLCANode.push_back(mTmpLCANode);
                    }
                    else{
                        vPathLCA[vDistance.size()-1].push_back(dLCA);
                        vPathLCANode[vDistance.size()-1].push_back(LCA);
                    }
                    //case 5
					
					if(i == vPathLCANode[pID].size())
					{
						vPathLCANode[pID][i] = vPathLCANode[pID][0];
					}
                    vector<float> vTmpdFix;
                    if(vSPTHeight[vPathLCANode[pID][i]] > vSPTHeight[eNodeID2])
                    {
                        float dFix = vSPTDistance[vPathDevPrime[pID]] - vSPTDistance[eNodeID2] + vPathFix[pID][i];
                        if(i == 0)
                        {
                            float dFix = vSPTDistance[vPathLCANode[pID][i]] - vSPTDistance[eNodeID2] + vPathFix[pID][0];
                            vTmpdFix.push_back(dFix);
                            vPathFix.push_back(vTmpdFix);
                        }
                        else{
                            if(vAncestor[vDistance.size()-1][i] == pID && bPath[pID]){
                                vPathFix[vDistance.size()-1].push_back(dFix);

							}
                            else{
                                float dFix = vSPTDistance[vPathLCANode[pID][i]] -vSPTDistance[eNodeID2] + vPathFix[pID][i];
                                vPathFix[vDistance.size()-1].push_back(dFix);
                            }
                        }
                    }

                    //other cases
                    else{
                        if(i == 0)
                        {
                            vTmpdFix.push_back(vPathFix[pID][0]);
                            vPathFix.push_back(vTmpdFix);
                        }
                        else{
                            if(vAncestor[vDistance.size()-1][i] == pID && bPath[pID])
                                vPathFix[vDistance.size()-1].push_back(vPathFix[pID][i] + vSPTDistance[vPathDevPrime[pID]] - vSPTDistance[eNodeID2]);
                            else
                                vPathFix[vDistance.size()-1].push_back(vPathFix[pID][i]);
                        }
                    }
                }

                vFather.push_back(pID);
                bPath.push_back(false);
                qPath.update(vDistance.size()-1, dist);

                reverse(vPath.begin(), vPath.end());
                reverse(vPathEdge.begin(), vPathEdge.end());
                unordered_map<int, int> mE;
                int i;


                //Pos stop at eNodeID1 as it is boundary of fixed
                for(i = 0; i < (int)vPath.size(); i++) 
				{
                    mE[vPath[i]] = i;
				}
                vmPathNodePos.push_back(mE);
                vPath.push_back(eNodeID2);
                vPathEdge.push_back(mineID);

                for(int j = vmPathNodePos[pID][eNodeID2]; j+1 < (int)vvPathCandidate[pID].size(); j++)
                {
                    int nodeID = vvPathCandidate[pID][j+1];
                    vPath.push_back(nodeID);
                    int edgeID = vvPathCandidateEdge[pID][j];
                    vPathEdge.push_back(edgeID);
                }
                vmArc.push_back(mArcTmp);
                vvPathCandidate.push_back(vPath);
                vvPathCandidateEdge.push_back(vPathEdge);
            }
		}

		if(countNumber >= 33836288){
			bCountNumber = false;
			break;
		}
    }

	if(!bCountNumber){
		cout << "Skip!" << endl;
		popPath = -1;
	}
	else{
		cout << "eKSPPrune countNumber: "<< countNumber << " Pop Path Length: " << popPath << endl; 
	}
    return -1;
}

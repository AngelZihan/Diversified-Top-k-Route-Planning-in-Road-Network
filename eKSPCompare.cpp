#include "graph.h"

int Graph::eKSPCompare(int ID1, int ID2, int k, vector<int>& kResults, vector<vector<int> >& vkPath, double t, int& countNumber, int& popPath, float& percentage, float &SimTime)
{
	percentage = 0;
    //Shortest Path Tree Info
	float countAncestor = 0;
	float countNonAncestor = 0;
    countNumber = 0;
    popPath = 0;
	bool bCountNumber = true;
    vector<int> vSPTDistance(nodeNum, INF);
    vector<int> vSPTParent(nodeNum, -1);
    vector<int> vSPTHeight(nodeNum, -1);
    vector<int> vSPTParentEdge(nodeNum, -1);
    vector<int> vSPTChildren(nodeNum, -1);
    vector<int> vTmp;
    vector<vector<int>> vSPT(nodeNum, vTmp); //Tree from root
    SPT(ID1, vSPTDistance, vSPTParent, vSPTHeight, vSPTParentEdge, vSPT);

    //LCA
    vector<vector<float> > vPathLCA;



    std::chrono::high_resolution_clock::time_point t1;
    std::chrono::high_resolution_clock::time_point t2;
    std::chrono::duration<double> time_span;
    t1 = std::chrono::high_resolution_clock::now();
    vector<vector<int> > RMQ = makeRMQ(ID1, vSPT, vSPTHeight,vSPTParent);
    t2 = std::chrono::high_resolution_clock::now(); 
    time_span = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1);
    cout << "RMQ Time:" << time_span.count() << endl;

	double time = 0;
	SimTime = 0;

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
    vector<int> dEdge;
    vector<bool> bPath;
    vector<vector<int> > vvPathCandidate;	 //nodes
    vector<vector<int> > vvPathCandidateEdge;//edges
    vector<unordered_map<int, int> > vmPathNodePos;	//Position of the non-fixed vertices
    //The size of mvPathNodePos indicates the fixed pos
    vector<vector<pair<int, int> > > vvPathNonTree;
    vector<multimap<int, int> > vmArc;
    vector<int> vFather;
	vAncestor.reserve(nodeNum*10); 
	vPathDevPrime.reserve(nodeNum*10);
	vPathLCANode.reserve(nodeNum*10);
	vPathFix.reserve(nodeNum*10);
	vPathLCA.reserve(nodeNum*10);
	mPathFix.reserve(nodeNum*10);
	vPathDeviation.reserve(nodeNum*10);
	bPath.reserve(nodeNum*10);
	dEdge.reserve(nodeNum*10);


	vector<unordered_set<int> > pResult;
    vFather.push_back(-1);

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
        vPath.push_back(p);
        for(int i = 0; i < (int)adjListEdgeR[oldP].size(); i++)
        {
            int eID = adjListEdgeR[oldP][i].second;
            if(eID != e)
                mArc.insert(make_pair(vEdgeReducedLength[eID], eID));
            //	else
            //		cout << "Skip " << e << endl;
        }
        oldP = p;
        vPathEdge.push_back(e);
        e = vSPTParentEdge[p];
        p = vSPTParent[p];
    }

    //cout << mArc.size() << endl;
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
    vector<int> vTmpAncestor;
    vTmpAncestor.push_back(-1);
    vAncestor.push_back(vTmpAncestor);
    vector<float> vTmpLCA;
    vTmpLCA.push_back(vSPTDistance[ID2]);
    vPathLCA.push_back(vTmpLCA);
    qPath.update(vvPathCandidate.size()-1, vSPTDistance[ID2]);

    vector<int> vResultID;
    int topPathID, topPathDistance;
    int pcount = 0;
    int oldDistance = -1; 
	bool bError = false;
    while((int)kResults.size() < k && !qPath.empty())
    {
       // cout << kResults.size() << " k: " << k << endl;
        float addLength = 0;
        pcount++;
		popPath++;
        qPath.extract_min(topPathID, topPathDistance);
       // cout << topPathID << "\t" << topPathDistance << endl;
        //cout << topPathID << endl;
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
            //popPath++;
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
            }
            else{
				t1 = std::chrono::high_resolution_clock::now(); 
                for (int i = 0; i < vResultID.size(); i++) {
                    bool vFind = false; 
                    for(int j = 0; j < vAncestor[topPathID].size(); j++)
                    {
                        if(vResultID[i] == vAncestor[topPathID][j])
                        {
							countAncestor += 1;
                            vFind = true;
							float simAdd = vPathFix[topPathID][j] + vPathLCA[topPathID][j] + mPathFix[i];
							//Sim 1
							//sim = simAdd / (kResults[i] + topPathDistance - simAdd);
					
							//Sim 2 
							//sim = simAdd / (2*kResults[i]) + simAdd / (2*topPathDistance);

							//Sim 3
							//sim = sqrt((simAdd*simAdd) / ((double)kResults[i]*(double)topPathDistance));
							
							//Sim 4
							/*int maxLength;
							if(addLength > topPathDistance)
								maxLength = simAdd;
							else
								maxLength = topPathDistance;
							sim = simAdd / maxLength;*/

							//Sim 5
							sim = simAdd / kResults[i];	
                            break;
                        }
                    }
					if(vFind == false)
					{
						countNonAncestor += 1;
						for(auto ie = vvPathCandidateEdge[topPathID].begin(); ie != vvPathCandidateEdge[topPathID].end(); ie++){
							if(pResult[i].find(*ie) != pResult[i].end())
								addLength += vEdge[*ie].length;
						}	
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

				t2 = std::chrono::high_resolution_clock::now(); 
				time_span = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1);
				SimTime += time_span.count(); 

                if (sim <= t) {
                    cout << "sim: " << sim << " topPath:" << topPathID << endl;
                    kResults.push_back(topPathDistance);
                    vvResult.push_back(vvPathCandidateEdge[topPathID]);
                    vkPath.push_back(vvPathCandidate[topPathID]);
                    vResultID.push_back(topPathID);
                    bPath[topPathID] = true;
                    mPathFix.push_back(vDistance[vFather[topPathID]] - vSPTDistance[vPathDeviation[topPathID]] + dEdge[topPathID]);
					unordered_set<int> pTmp2;
					for(auto ie = vvPathCandidateEdge[topPathID].begin(); ie != vvPathCandidateEdge[topPathID].end(); ie++)
					{
						pTmp2.insert(*ie);
					}
					pResult.push_back(pTmp2);
                }
            }
        }

        vector<int> vTwo;
        vTwo.push_back(topPathID);
        if(vFather[topPathID] != -1 &&  !vmArc[vFather[topPathID]].empty())
            vTwo.push_back(vFather[topPathID]);
        for(auto& pID : vTwo)
        {
            bool bLoop = true;
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

                //eNodeID1 is also the first point from the deviation edge
                int eNodeID1 = vEdge[mineID].ID1;
                int eNodeID2 = vEdge[mineID].ID2;

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
                vPath.clear();
                vPathEdge.clear();
                int p = eNodeID1;
                int e = vSPTParentEdge[p];

                while(p != -1)
                {
					sE.insert(p);
                    vPath.push_back(p);
                    for(int i = 0; i < (int)adjListEdgeR[p].size(); i++)
                    {
                        int reID = adjListEdgeR[p][i].second;
                        int eID1 = vEdge[reID].ID1;
                        int dTmp = distTmp + vEdge[reID].length;

                        if(sE.find(eID1) != sE.end())
                            continue;

                        if(reID != e && reID != mineID)
                            mArcTmp.insert(make_pair(vEdgeReducedLength[reID], reID));
                    }

                    vPathEdge.push_back(e);
                    distTmp += vEdge[e].length;

                    p = vSPTParent[p];
                    if(vSPTParent[p] == -1)
                    {
                        vPath.push_back(p);
                        break;
                    }
                    e = vSPTParentEdge[p];
                }
                dEdge.push_back(vEdge[mineID].length);
                int dist = vDistance[pID] - vSPTDistance[eNodeID2] + vSPTDistance[eNodeID1] + vEdge[mineID].length;
                vDistance.push_back(dist);
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
				
				t1 = std::chrono::high_resolution_clock::now();
                for(int i = 0; i < vAncestor[vDistance.size()-1].size(); i++)
                {
                    //LCA
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
                    vector<float> vTmpdFix;
					if(i == vPathLCANode[pID].size())
					{
						vPathLCANode[pID][i] = vPathLCANode[pID][0];
					}
                    if(vSPTHeight[vPathLCANode[pID][i]] > vSPTHeight[eNodeID2])
                    {
                        int dFix = vSPTDistance[vPathDevPrime[pID]] - vSPTDistance[eNodeID2] + vPathFix[pID][i];
                        if(i == 0)
                        {
                            int dFix = vSPTDistance[vPathLCANode[pID][i]] - vSPTDistance[eNodeID2] + vPathFix[pID][0];
                            vTmpdFix.push_back(dFix);
                            vPathFix.push_back(vTmpdFix);
                        }
                        else{
							if(vAncestor[vDistance.size()-1][i] == pID && bPath[pID])
								vPathFix[vDistance.size()-1].push_back(dFix);
							else{
								int dFix = vSPTDistance[vPathLCANode[pID][i]] -vSPTDistance[eNodeID2] + vPathFix[pID][i];
								vPathFix[vDistance.size()-1].push_back(dFix);
							}
                        }
                    }
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
				t2 = std::chrono::high_resolution_clock::now();
				time_span = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1);
				time += time_span.count(); 
                vFather.push_back(pID);
                bPath.push_back(false);
                qPath.update(vDistance.size()-1, dist);

                reverse(vPath.begin(), vPath.end());
                reverse(vPathEdge.begin(), vPathEdge.end());
                unordered_map<int, int> mE;
                int i;
                //Pos stop at eNodeID1 as it is boundary of fixed
                for(i = 0; i < (int)vPath.size(); i++)
                    mE[vPath[i]] = i;
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
		percentage = countAncestor/(countNonAncestor + countAncestor);
		cout << "Sim Time:" << SimTime << endl;
		cout << "eKSPCompare countNumber: "<< countNumber << " Pop Path: " << popPath << " Percentage: " << percentage << endl;
	}
    return -1;
}

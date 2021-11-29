#include "graph.h"

int Graph::eKSPNew(int ID1, int ID2, int k, vector<int>& kResults, vector<vector<int> >& vkPath, double t, int& countNumber, int& popPath, vector<float>& sim, float &SimTime)
{
    //Shortest Path Tree Info
    countNumber = 0;		
	bool bCountNumber = true;
    popPath = 0;
	int popPathTest = 0;
    vector<int> vSPTDistance(nodeNum, INF);
    vector<int> vSPTParent(nodeNum, -1);
    vector<int> vSPTHeight(nodeNum, -1);
    vector<int> vSPTParentEdge(nodeNum, -1);
    vector<int> vTmp;
    vector<vector<int> > vSPT(nodeNum, vTmp); //Tree from root
    SPT(ID1, vSPTDistance, vSPTParent,vSPTHeight, vSPTParentEdge, vSPT);

    vector<int> vEdgeReducedLength(vEdge.size(), INF);
    for(int i = 0; i < (int)vEdge.size(); i++)
        vEdgeReducedLength[i] = vEdge[i].length + vSPTDistance[vEdge[i].ID1] - vSPTDistance[vEdge[i].ID2];

    vector<vector<int> > vvResult;	//Exact Path
    vvResult.reserve(k);
    vector<int> vDistance;
    vector<int> vPathParent;	//Deviated from
    vector<int> vPathParentPos;	//Deviated Pos from Parent
    vector<vector<int> > vvPathCandidate;	 //nodes
    vector<vector<int> > vvPathCandidateEdge;//edges
    vector<unordered_map<int, int> > vmPathNodePos;	//Position of the non-fixed vertices
    //The size of mvPathNodePos indicates the fixed pos
    vector<vector<pair<int, int> > > vvPathNonTree;
    vector<multimap<int, int> > vmArc;
    vector<int> vFather;
    vFather.push_back(-1);

	vector<unordered_set<int> > pResult; 
    std::chrono::high_resolution_clock::time_point t1;
    std::chrono::high_resolution_clock::time_point t2;
    std::chrono::duration<double> time_span;
	SimTime = 0;

    //float sim;
	double sim1;

    /*label lTmp;

    lTmp.pre = -1;
    lTmp.post = -1;
    lTmp.parent = -1;
    vector<label> vLabel(nodeNum, lTmp);
    intervalLabel(ID1, vSPT, vSPTParent, vLabel); //ID1 is root*/

    vector<int> vPath;
    vector<int> vPathEdge;
    vPath.push_back(ID2);
    vector<pair<int, int> > vPathNonTree;
    int oldP = ID2;
    int p = vSPTParent[ID2];
    int e = vSPTParentEdge[ID2];
    multimap<int, int> mArc;
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
    vmArc.push_back(mArc);
    //cout << mArc.size() << endl;
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
    vPathParent.push_back(-1);
    vPathParentPos.push_back(0);
    qPath.update(vvPathCandidate.size()-1, vSPTDistance[ID2]);

    vector<int> vResultID;
    int topPathID, topPathDistance;
    int pcount = 0;
    int oldDistance = -1;
    while((int)kResults.size() < k && !qPath.empty())
    {
        float addLength = 0;
        pcount++;
		popPath++;
        qPath.extract_min(topPathID, topPathDistance);
        //cout << topPathID << "...." << topPathDistance << endl;
        if(topPathDistance < oldDistance)
            cout<< "Error" <<endl;
        oldDistance = topPathDistance;
        // cout << endl << pcount << "\t" << topPathDistance  << "\t" << kResults.size() << endl;

        //Loop Test
        unordered_set<int> us;
        bool bTopLoop = false;
        for(auto& v : vvPathCandidate[topPathID])
        {
            if(us.find(v) == us.end())
                us.insert(v);
            else
            {
				//cout << topPathID << endl;
				//cout << v << endl; 
                bTopLoop = true;
                //cout << "Top Loop!" << endl;
                break;
            }
        }
        if(!bTopLoop)
        {
		   popPathTest++;
           if (vvResult.size() == 0)
            {
                vvResult.push_back(vvPathCandidateEdge[topPathID]);
                kResults.push_back(topPathDistance);
                vkPath.push_back(vvPathCandidate[topPathID]);
                vResultID.push_back(topPathID);
				unordered_set<int> pTmp;
				for(auto ie = vvPathCandidateEdge[topPathID].begin(); ie != vvPathCandidateEdge[topPathID].end(); ie++)
				{
					pTmp.insert(*ie);
				}
				pResult.push_back(pTmp); 
            }
            else {
                //for (int i = 0; i < vvResult.size(); i++)
				t1 = std::chrono::high_resolution_clock::now(); 
				for(int i = 0; i < pResult.size(); i++)
                {
                //for (auto m = vvResult[i].begin(); m != vvResult[i].end(); m++) {
					for (auto ie = vvPathCandidateEdge[topPathID].begin(); ie != vvPathCandidateEdge[topPathID].end(); ie++) {
						if(pResult[i].find(*ie) != pResult[i].end()){
                        //if (*ie == *m) {
							addLength += vEdge[*ie].length;
                        }
                    }
                    //}
					//Sim 1
                    //sim1 = addLength / (kResults[i] + topPathDistance - addLength);
					
					//Sim 2 
					//sim1 = addLength / (2*kResults[i]) + addLength / (2*topPathDistance);

					//Sim 3
					//sim1 = sqrt((addLength*addLength) / ((double)kResults[i]*(double)topPathDistance)); 
//					cout << addLength * addLength << "\t" << (double)kResults[i] * (double)topPathDistance << "\t" << kResults[i] << "\t" << topPathDistance << "\t" << (addLength*addLength) / (kResults[i]*topPathDistance) << endl;  
//					cout << sim1 << endl;	
					//Sim 4
					/*int maxLength;
					if(addLength > topPathDistance)
						maxLength = addLength;
					else
						maxLength = topPathDistance;
					sim1 = addLength / maxLength;*/

					//Sim 5
					sim1 = addLength / kResults[i];
					
                    addLength = 0;
                    if (sim1 > t)
                        break;
                }


				t2 = std::chrono::high_resolution_clock::now(); 
				time_span = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1);
				SimTime += time_span.count(); 
                if (sim1 <= t) {
					//cout << topPathID << endl;
                    cout << "sim: " << sim1 << " topPathID: " << topPathID<< endl;
                    kResults.push_back(topPathDistance);
                    vvResult.push_back(vvPathCandidateEdge[topPathID]);
                    vkPath.push_back(vvPathCandidate[topPathID]);
                    vResultID.push_back(topPathID);
					unordered_set<int> pTmp2;
					for(auto ie = vvPathCandidateEdge[topPathID].begin(); ie != vvPathCandidateEdge[topPathID].end(); ie++)
					{
						pTmp2.insert(*ie);
					}
					pResult.push_back(pTmp2);
					//vResultThreshold.push_back(vDistance[topPathID] * t);
                }
            }

			/*vvResult.push_back(vvPathCandidateEdge[topPathID]);
            kResults.push_back(topPathDistance);
            vkPath.push_back(vvPathCandidate[topPathID]);
            vResultID.push_back(topPathID);

            for (auto m = vvResult[0].begin(); m != vvResult[0].end(); m++) {
                for (auto ie = vvPathCandidateEdge[topPathID].begin();
                    ie != vvPathCandidateEdge[topPathID].end(); ie++) {
					if (*ie == *m) {
						addLength += vEdge[*ie].length;
                    }
                }
			}
			sim.push_back(addLength / kResults[0]);
			//cout << "sim: " << sim << " topPathID: " << topPathID << endl;
			//popPath++;
//			cout << kResults.size()  << endl;*/
        }
        vector<int> vTwo;
        vTwo.push_back(topPathID);
        //cout << vFather[topPathID] << endl;
        if(vFather[topPathID] != -1 &&  !vmArc[vFather[topPathID]].empty())
            vTwo.push_back(vFather[topPathID]);
        //cout << topPathID << "...." << vFather[topPathID] << endl;
        //int count = 0;
        for(auto& pID : vTwo)
        {
            //cout << pID << endl;
            //count++;
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
				
				//eNodeID1 is the deviation dege
                int eNodeID1 = vEdge[mineID].ID1;
                int eNodeID2 = vEdge[mineID].ID2;

                //Loop in the fixed part, prune
                bool bFixedLoop = false;
                unordered_set<int> sE;
                for(int i = vmPathNodePos[pID][eNodeID2]; i < (int)vvPathCandidate[pID].size(); i++)
                {
                   if(sE.find(vvPathCandidate[pID][i]) == sE.end())
                        sE.insert(vvPathCandidate[pID][i]);
                    else
                    {
                        bFixedLoop = true;
                        //cout << "Loop !" << endl;
                        break;
                    }
					/*if(vvPathCandidate[pID][i] == eNodeID1)
					{
						//cout << "loop" << endl;
						bFixedLoop = true;
						break;
					}*/
                   /* if(vLabel[eNodeID1].pre < vLabel[vvPathCandidate[pID][i]].pre || vLabel[eNodeID1].post > vLabel[vvPathCandidate[pID][i]].post)
                        continue;
                    else
                    {
						bFixedLoop = true;
						break;
					}
                     */  
                }

                if(bFixedLoop)
                    continue;
                //countNumber += 1;
                int distTmp = vDistance[pID] - vSPTDistance[eNodeID2] + vEdge[mineID].length;
                bLoop = false;


//				cout << "eNodeID1:" << eNodeID1 << "\t" << "eNodeID2:" << eNodeID2 << endl;
//				cout << "Previous:" << vvPathCandidate[pID][vmPathNodePos[pID][eNodeID2]-1] << endl;

                //Traverse the non-fixed part: the tree
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

						//loop Test
                        /*bool fLoop = false;
                        for(int i = vmPathNodePos[pID][eNodeID2]; i < (int)vvPathCandidate[pID].size(); i++)
                        {
                            if(vLabel[eID1].pre < vLabel[vvPathCandidate[pID][i]].pre || vLabel[eID1].post > vLabel[vvPathCandidate[pID][i]].post)
                                continue;
                            else
                            {
                                fLoop = true;
                                break;
                            }
                        }
                        if(fLoop == true)
                            continue;*/
                        if(sE.find(eID1) != sE.end()) //Loop Test
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

                int dist = vDistance[pID] - vSPTDistance[eNodeID2] + vSPTDistance[eNodeID1] + vEdge[mineID].length;
//				cout << "New Dist:" << dist << endl << endl;
                vDistance.push_back(dist);
                vFather.push_back(pID);
                //countNumber += 1;
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
                //cout << mArcTmp.size() << endl;
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
	else
	{
		cout << "Sim Time:" << SimTime << endl;
		cout << "eKSP countNumber: "<< countNumber << " Pop Path: " << popPath << endl;
	}
	//cout << " Pop Path Test: " << popPathTest << endl;
    return -1;
}

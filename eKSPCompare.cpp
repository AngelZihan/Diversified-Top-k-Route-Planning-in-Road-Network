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
                //bPath[topPathID] = true;
                mPathFix.push_back(0);
                //cout << topPathID << endl;
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
//					cout << "vAncestor size:" << vAncestor[topPathID].size() << endl;
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
					
                            //sim = simAdd / kResults[i];
							//cout << mPathFix[i] << endl;
                            //cout << (vPathLCA[topPathID] + vPathFix[topPathID][n] + mPathFix[i]) << endl;
                            /*float addLengthTest;
                            for (auto m = vvResult[i].begin(); m != vvResult[i].end(); m++) {
                                for (auto ie = vvPathCandidateEdge[topPathID].begin();
                                     ie != vvPathCandidateEdge[topPathID].end(); ie++) {
                                    if (*ie == *m) {
                                        addLengthTest += vEdge[*ie].length;
                                    }
                                }
                            }
                            if(addLengthTest != vPathLCA[topPathID][j] + vPathFix[topPathID][j] + mPathFix[i]) {
                                cout << "addLengthTest " << addLengthTest << " addLength: " << simAdd << " "
                                     << vPathLCA[topPathID][j] << " " << vPathFix[topPathID][j] << " " << mPathFix[i] << " pathID:"
                                     << topPathID << " AncestorID:" << vAncestor[topPathID][j] << " j: " << j<< endl;
                                //cout << vAncestor[164].size() << endl;
                                if(vFather[topPathID] == vAncestor[topPathID][i])
                                    cout << "Father!!!" << endl;
								bError = true; 
								
								addLengthTest = 0; 
								int p1length = 0;
								int ancestorID = vAncestor[topPathID][j];
								for (auto ie = vvPathCandidateEdge[topPathID].begin(); ie != vvPathCandidateEdge[topPathID].end(); ie++) 
								 {  
										p1length += vEdge[*ie].length;
										bool bFound = false; 
										for (auto m = vvResult[i].begin(); m != vvResult[i].end(); m++)  
										{ 
											if (*ie == *m) {
												addLengthTest += vEdge[*m].length;   
												bFound = true;
												break;
											}
										}
										if(bFound)
											cout << "Same " << *ie << "\t" << vEdge[*ie].length << endl;
										else
											cout << "Different " << *ie << "\t" << vEdge[*ie].length << endl; 
								 }
								 cout << "pAncestor:" << vDistance[ancestorID] << "\t" << addLengthTest << "\t" << p1length << endl;  

								cout << "Father Information: Fixed:" << vPathFix[vFather[topPathID]][j] <<  endl;   
								if(bPath[vFather[topPathID]])
									cout << "Father in Result" << endl;
								else
									cout << "Father Not in Result" << endl;
								addLengthTest = 0; 
								p1length = 0;
								for (auto ie = vvPathCandidateEdge[vFather[topPathID]].begin(); ie != vvPathCandidateEdge[vFather[topPathID]].end(); ie++) 
								 {  
										p1length += vEdge[*ie].length;
										bool bFound = false; 
										for (auto m = vvResult[i].begin(); m != vvResult[i].end(); m++) 
										{ 
											if (*ie == *m) {
												//addLengthTest += vEdge[*ie].length; 
												addLengthTest += vEdge[*m].length;   
												bFound = true;
											//	break;
											}
										}
									
										if(bFound)
											cout << "Same " << *ie << "\t" << vEdge[*ie].length << endl;
										else
											cout << "Different " << *ie << "\t" << vEdge[*ie].length << endl;  
								}
								 cout << "pFather:" << vDistance[vFather[topPathID]] << "\t" << addLengthTest << "\t" << p1length << endl;   

								 for(auto& nid : vvPathCandidate[vFather[topPathID]]) 
									 cout << nid << endl;
								
								 cout << endl << "Son "<< endl;
								 for(auto& nid : vvPathCandidate[topPathID]) 
									 cout << nid << endl;

							}
                                addLengthTest = 0;*/
							/*int testLCA;
							int testLCALength;
							for(int m = 0; m < vvResult[i].size(); m++){
								if(vvResult[i][m] != vvPathCandidateEdge[topPathID][m]){
									testLCA = vEdge[vvResult[i][m]].ID1;
									testLCALength = vSPTDistance[testLCA];
									break;
								}
							}*/

							/*if(testLCALength == vPathLCA[topPathID][j])	
                                cout << "testLCALength " << testLCALength << " vPathLCA: " << vPathLCA[topPathID][j] << endl;*/

							/*int testmPathFix;
							int testmPathFixLength;
							int testaddFixLength = 0;
							for(int n = 0; n < vmPathNodePos[vResultID[i]][vPathDevPrime[vResultID[i]]]; n++){
								testaddFixLength += vEdge[vvResult[i][n]].length;
							}
							
							testaddFixLength = vDistance[vResultID[i]] - testaddFixLength;

							if(testaddFixLength != mPathFix[i] )
                                cout << "testaddFixLength " << testaddFixLength << " mPathFix: " << mPathFix[i] << endl;
							testaddFixLength = 0;*/
							/*for(int k = vmPathNodePos[vvResult[i][testLCA]]; k < vmPathNodePos[vvResult[i][vPathDevPrime[vResultID[i]]]]; k++){
								if(vvResult[i])
							}*/
                            /*int New = vPathLCA[topPathID][j] + vPathFix[topPathID][j] + mPathFix[i];
                            if(addLengthTest != New)
                            {
                                cout << "addlengthTest " << addLengthTest << " New: " << New << " i: " << i << endl;
                                cout << "False " << topPathID << endl;
                            }*/
                            /*else
                                cout << "Ture" << endl;*/
                            //cout << "" << vPathLCA[topPathID] + vPathFix[topPathID][n] + mPathFix[i] << " addLengthTest: " << addLengthTest<< endl;
                            //addLengthTest = 0;
                            //n++;
                            break;
                        }
                    }
                    /*if(vFind == false)
                    {
						countNonAncestor += 1;
                        for (auto m = vvResult[i].begin(); m != vvResult[i].end(); m++) {
                            for (auto ie = vvPathCandidateEdge[topPathID].begin();
                                 ie != vvPathCandidateEdge[topPathID].end(); ie++) {
                                if (*ie == *m) {
                                    addLength += vEdge[*ie].length;
                                }
                            }
                        }
                        //sim = addLength / kResults[i];

						//Sim 1
						sim = addLength / (kResults[i] + topPathDistance - addLength);
						//Sim 2 
						//sim = addLength / (2*kResults[i]) + addLength / (2*topPathDistance);

						//Sim 3
						//sim = sqrt((addLength*addLength) / ((double)kResults[i]*(double)topPathDistance));
							
						//Sim 4
						//int maxLength;
						//if(addLength > topPathDistance)
						//	maxLength = addLength;
						//else
						//	maxLength = topPathDistance;
						//sim = addLength / maxLength;

						//Sim 5
						//sim = addLength / kResults[i];
                        addLength = 0;
                    }*/
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

                    /*for (auto m = vvResult[i].begin(); m != vvResult[i].end(); m++) {
                        for (auto ie = vvPathCandidateEdge[topPathID].begin();
                             ie != vvPathCandidateEdge[topPathID].end(); ie++) {
                            if (*ie == *m) {
                                addLength += vEdge[*ie].length;
                            }
                        }
                    }
                    sim = addLength / kResults[i];
                    addLength = 0;*/
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
                    //cout << dEdge[topPathID] << endl;
                    mPathFix.push_back(vDistance[vFather[topPathID]] - vSPTDistance[vPathDeviation[topPathID]] + dEdge[topPathID]);
                    //cout << topPathID << "..." << bPath[topPathID] << endl;	
					unordered_set<int> pTmp2;
					for(auto ie = vvPathCandidateEdge[topPathID].begin(); ie != vvPathCandidateEdge[topPathID].end(); ie++)
					{
						pTmp2.insert(*ie);
					}
					pResult.push_back(pTmp2);
                }
            }
        }

//		if(bError)
//			break;

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
                        //cout << vAncestor[vDistance.size()-1].size() << " pID:" << vAncestor[vDistance.size()-1][vAncestor[vDistance.size()-1].size()-1]<< endl;
                    }
                }
                else{
                    vAncestor.push_back(vAncestor[pID]);
                }
                //cout << vDistance.size() << "  " << vAncestor[vDistance.size()-1].size() << endl;
               // cout << "last vancestor size:" << vAncestor[vDistance.size()-1].size() << endl;
				
				t1 = std::chrono::high_resolution_clock::now();
                for(int i = 0; i < vAncestor[vDistance.size()-1].size(); i++)
                {
                    //LCA
                    int _p, _q;
                    //cout << vPathDevPrime[vAncestor[vDistance.size()-1][i]] << endl;
                    //_p = rEulerSeq[vPathDevPrime[0]];
                    //cout << vAncestor[vDistance.size()-1][i] << endl;
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
                    //cout << i << "   " << vAncestor[vDistance.size()-1][i] << endl;
                    vector<float> vTmpdFix;
                    //cout << vSPTHeight[vPathLCANode[pID][i]] << "..." << vSPTHeight[eNodeID2]<< endl;
                    //cout << vSPTHeight[eNodeID2]<< endl;
                    //cout << vPathLCANode[pID][i] << "i: " << i << " size:" << vPathLCANode[pID].size() << endl;
					if(i == vPathLCANode[pID].size())
					{
						//cout << "Here!" << endl;
						vPathLCANode[pID][i] = vPathLCANode[pID][0];
					}
                    if(vSPTHeight[vPathLCANode[pID][i]] > vSPTHeight[eNodeID2])
                    {
                        int dFix = vSPTDistance[vPathDevPrime[pID]] - vSPTDistance[eNodeID2] + vPathFix[pID][i];
						//int dFix = vSPTDistance[vPathDevPrime[pID]] - vSPTDistance[eNodeID2] + vPathFix[pID][i] + dEdge[pID];
                        //cout << dFix << endl;
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
                            //cout << vPathFix[vDistance.size()-1][i] << endl;
                        }
                    }
                   /* else if( vSPTHeight[vPathDevPrime[pID]] > vSPTHeight[eNodeID2] && vSPTHeight[LCA]  > vSPTHeight[vPathLCANode[pID]])
                    {
                        if(i == 0)
                        {
                            vTmpdFix.push_back(vPathFix[pID][0]);
                            vPathFix.push_back(vTmpdFix);
                        }
                        // middle part How to calculate?????
                        else{
                            _p = rEulerSeq[vPathDevPrime[pID]];
                            _q = rEulerSeq[eNodeID1];
                            int mLCANode = LCAQuery(_p, _q, vSPT,  vSPTHeight,vSPTParent);
                            int mLCA = vSPTDistance[LCA] - vSPTDistance[vPathLCA[vDistance.size()-1]];
                            vPathFix[vDistance.size()-1].push_back(vPathFix[pID][i] + vSPTDistance[vPathDevPrime[pID]] - vSPTDistance[eNodeID2] + dEdge[pID] + mLCA);
                        }
                    }*/
                    else{
                        if(i == 0)
                        {
                            vTmpdFix.push_back(vPathFix[pID][0]);
                            vPathFix.push_back(vTmpdFix);
                        }
                        else{
							if(vAncestor[vDistance.size()-1][i] == pID && bPath[pID])
                            //vPathFix[vDistance.size()-1].push_back(vPathFix[pID][i] + vSPTDistance[vPathDevPrime[pID]] - vSPTDistance[eNodeID2] + dEdge[pID]);
								vPathFix[vDistance.size()-1].push_back(vPathFix[pID][i] + vSPTDistance[vPathDevPrime[pID]] - vSPTDistance[eNodeID2]);
							else
								vPathFix[vDistance.size()-1].push_back(vPathFix[pID][i]);
							//vPathFix[vDistance.size()-1].push_back(vPathFix[pID][i]);
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
		//cout << "Organizing time:" << time << endl;
		cout << "Sim Time:" << SimTime << endl;
		//cout << "countAncestor: " << countAncestor << " countNonAncestor: " << countNonAncestor << endl;
		cout << "eKSPCompare countNumber: "<< countNumber << " Pop Path: " << popPath << " Percentage: " << percentage << endl;
	}
    return -1;
}

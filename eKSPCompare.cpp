#include "graph.h"

int Graph::eKSPCompare(int ID1, int ID2, int k, vector<int>& kResults, vector<vector<int> >& vkPath, double t, int& countNumber, int& popPath, float& percentage, float& SimTime, float& AveSim, float& minSim, float& maxSim)
{
	AveSim = 0;
	minSim = 1;
	maxSim = 0;
    std::chrono::high_resolution_clock::time_point t1;
    std::chrono::high_resolution_clock::time_point t2;
    std::chrono::duration<double> time_span;
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

    SPT(ID1, ID2, vSPTDistance, vSPTParent, vSPTHeight, vSPTParentEdge, vSPT);

    //LCA
    vector<vector<float> > vPathLCA;
    vector<vector<int> > RMQ = makeRMQ(ID1, vSPT, vSPTHeight,vSPTParent);
    double time = 0;
    SimTime = 0;
    double edgeTime = 0;
	double sETime = 0;
    double pathTime = 0;
    double loopTime = 0;

    vector<int> vEdgeReducedLength(vEdge.size(), INF);
    for(int i = 0; i < (int)vEdge.size(); i++)
        vEdgeReducedLength[i] = vEdge[i].length + vSPTDistance[vEdge[i].ID1] - vSPTDistance[vEdge[i].ID2];

    vector<vector<int> > vvResult;	//Exact Path
    vvResult.reserve(k);
    vector<int> vDistance;

    //Open
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
	vector<set<int>> vvsE;
    vector<vector<pair<int, int> > > vvPathNonTree;
    vector<vector<pair<int, int> > > vvmArc;
    vector<vector<pair<int, int> > > vvmPathNodePos;
    vector<int> vFather;

    vector<unordered_set<int> > pResult;
    vFather.push_back(-1);

    float sim;

    benchmark::pHeap<3, int, int, int> Arc(2*nodeNum);
    vector<int> vPath;
    vector<int> vPathEdge;
    vPath.push_back(ID2);
    vector<pair<int, int> > vPathNonTree;
    int p = vSPTParent[ID2];
    int e = vSPTParentEdge[ID2];
    float sim1;
    float addLength1 = 0;
    int oldP = ID2;
    while(p != -1)
    {
        for(int i = 0; i < (int)adjListEdgeR[oldP].size(); i++)
        {
            int eID = adjListEdgeR[oldP][i].second;
            if(eID != e){
                Arc.update(eID,vEdgeReducedLength[eID],vPath.size()-1);
            }
        }
        vPath.push_back(p);
        oldP = p;
        vPathEdge.push_back(e);
        e = vSPTParentEdge[p];
        p = vSPTParent[p];
    }
    int mmArcEdgeID, mmArcReducedLength, mmArcPosition;
    vector<pair<int, int> > mmArc;
    vector<pair<int, int> > mmPos;
    while(!Arc.empty())
    {
        Arc.extract_min(mmArcEdgeID, mmArcReducedLength, mmArcPosition);
        mmArc.push_back(make_pair(mmArcEdgeID,mmArcReducedLength));
        mmPos.push_back(make_pair(mmArcEdgeID,mmArcPosition));
    }
    vvmArc.push_back(mmArc);
    vvmPathNodePos.push_back(mmPos);

    reverse(vPath.begin(), vPath.end());
    reverse(vPathEdge.begin(), vPathEdge.end());

    benchmark::heap<2, int, int> qPath(nodeNum);
    vvPathCandidate.push_back(vPath);
    vvPathCandidateEdge.push_back(vPathEdge);
    vDistance.push_back(vSPTDistance[ID2]);

    //Open
    bPath.push_back(true);
    dEdge.push_back(0);
    vPathParent.push_back(-1);
    vPathParentPos.push_back(0);
    vPathDeviation.push_back(ID2);


    vector<int> vTmpLCANode;
    vTmpLCANode.push_back(ID2);

    //Open
    vPathLCANode.push_back(vTmpLCANode);
    vPathDevPrime.push_back(ID2);


    vector<float> vTmpFix;
    vTmpFix.push_back(0);

    //Open
    vPathFix.push_back(vTmpFix);


    vector<int> vTmpAncestor;
    vTmpAncestor.push_back(-1);

    //Open
    vAncestor.push_back(vTmpAncestor);

    vector<float> vTmpLCA;
    vTmpLCA.push_back(vSPTDistance[ID2]);

    //Open
    vPathLCA.push_back(vTmpLCA);

    qPath.update(vvPathCandidate.size()-1, vSPTDistance[ID2]);

    vector<int> vResultID;
    int topPathID, topPathDistance;
    int pcount = 0;
    int oldDistance = -1;
    bool bError = false;

    while((int)kResults.size() < k && !qPath.empty())
    {
        float addLength = 0;
        pcount++;
        popPath++;
        qPath.extract_min(topPathID, topPathDistance);
        if(topPathDistance < oldDistance)
            cout<< "Error" <<endl;
        oldDistance = topPathDistance;
        //Loop Test

        //Change to vector
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

        t1 = std::chrono::high_resolution_clock::now();
        if(!bTopLoop)
        {

			float simCount = 0;
            int n = 0;
            if (vvResult.size() == 0)
            {
                vvResult.push_back(vvPathCandidateEdge[topPathID]);
                kResults.push_back(topPathDistance);
                vkPath.push_back(vvPathCandidate[topPathID]);
                vResultID.push_back(topPathID);

                //Open
                mPathFix.push_back(0);

                unordered_set<int> pTmp;
                for(auto ie = vvPathCandidateEdge[topPathID].begin(); ie != vvPathCandidateEdge[topPathID].end(); ie++)
                {
                    pTmp.insert(*ie);
                }
                pResult.push_back(pTmp);
            }
            else{
				float simMax = 0;
				float simMin = 1;
                for (int i = 0; i < vResultID.size(); i++) {
                    bool vFind = false;

                    //Open
                    for(int j = 0; j < vAncestor[topPathID].size(); j++)
                    {
                        if(vResultID[i] == vAncestor[topPathID][j])
                        {
                            countAncestor += 1;
                            vFind = true;
                            float simAdd = vPathFix[topPathID][j] + vPathLCA[topPathID][j] + mPathFix[i];
                            //Sim 1
							sim = simAdd / (kResults[i] + topPathDistance - simAdd);

                            //Sim 2
                            //sim = simAdd / (2*kResults[i]) + simAdd / (2*topPathDistance);

                            //Sim 3
							//sim = sqrt((simAdd*simAdd) / ((double)kResults[i]*(double)topPathDistance));

                            //Sim 4
							//sim = simAdd / topPathDistance;


                            //Sim 5
							//sim = simAdd / kResults[i];
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
                        sim = addLength / (kResults[i] + topPathDistance - addLength);
                        //Sim 2
                        //sim = addLength / (2*kResults[i]) + addLength / (2*topPathDistance);

                        //Sim 3
                        //sim = sqrt((addLength*addLength) / ((double)kResults[i]*(double)topPathDistance));

                        //Sim 4
						//sim = addLength/ topPathDistance;


                        //Sim 5
                        //sim = addLength / kResults[i];
                        addLength = 0;
                    }
					simCount += sim;
                    if (sim > simMax)
                        simMax = sim;
					if(sim < simMin)
						simMin = sim;

                }
				if(simMax <= t){
                    if(simMax > maxSim)
						maxSim = simMax;
					if(simMin < minSim)
						minSim = simMin;
					AveSim += simCount;
                    kResults.push_back(topPathDistance);
                    vvResult.push_back(vvPathCandidateEdge[topPathID]);
                    vkPath.push_back(vvPathCandidate[topPathID]);
                    vResultID.push_back(topPathID);

                    //Open
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
        t2 = std::chrono::high_resolution_clock::now();
        time_span = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1);
        SimTime += time_span.count();

        vector<int> vTwo;
        vTwo.push_back(topPathID);
        if(vFather[topPathID] != -1 &&  !vvmArc[vFather[topPathID]].empty())
            vTwo.push_back(vFather[topPathID]);
        for(auto& pID : vTwo)
        {
            bool bLoop = true;
            while(bLoop)
            {
                if(vvmArc[pID].empty())
                {
                    vvPathCandidate[pID].clear();
                    vvPathCandidateEdge[pID].clear();
                    break;
                }
                int mineID;
                int eReducedLen;
                auto it = vvmArc[pID].begin();
                eReducedLen = (*it).second;
                mineID = (*it).first;
                countNumber += 1;
                vvmArc[pID].erase(it);

                //eNodeID1 is also the first point from the deviation edge
                int eNodeID1 = vEdge[mineID].ID1;
                int eNodeID2 = vEdge[mineID].ID2;

                bool bFixedLoop = false;

                //Change to vector
                auto itp = vvmPathNodePos[pID].begin();
                int ReID2Pos = (*itp).second;
                int eID2Pos = vvPathCandidate[pID].size()-ReID2Pos-1;
                vvmPathNodePos[pID].erase(itp);
				unordered_set<int> sE;
				for(int i = eID2Pos; i < (int) vvPathCandidate[pID].size(); i++)
                {
                    if(sE.find(vvPathCandidate[pID][i]) == sE.end())
                        sE.insert(vvPathCandidate[pID][i]);
                    else
                    {
                        bFixedLoop = true;
                        break;
                    }
                }

                if(bFixedLoop) {
                    continue;
                }
				//vvsE.push_back(sE);
                int distTmp = vDistance[pID] - vSPTDistance[eNodeID2] + vEdge[mineID].length;
                bLoop = false;

                //Open
                vPathDevPrime.push_back(eNodeID1);
                vPathDeviation.push_back(eNodeID2);
                vector<pair<int, int> > mmArcTmp;
                vPath.clear();
                vPathEdge.clear();
                int p = eNodeID1;
                int e = vSPTParentEdge[p];
                Arc.clear();
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
                            Arc.update(reID,vEdgeReducedLength[reID],vPath.size()+ReID2Pos);
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
                vector<pair<int, int> > mmE;
                while(!Arc.empty())
                {
                    Arc.extract_min(mmArcEdgeID, mmArcReducedLength, mmArcPosition);
					mmArcTmp.push_back(make_pair(mmArcEdgeID,mmArcReducedLength));
                    mmE.push_back(make_pair(mmArcEdgeID,mmArcPosition));
                }
				//Open
                dEdge.push_back(vEdge[mineID].length);

                int dist = vDistance[pID] - vSPTDistance[eNodeID2] + vSPTDistance[eNodeID1] + vEdge[mineID].length;
                vDistance.push_back(dist);

                //Open
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
                vFather.push_back(pID);

                //Open
                bPath.push_back(false);
                qPath.update(vDistance.size()-1, dist);

                //rbegin
                reverse(vPath.begin(), vPath.end());
                reverse(vPathEdge.begin(), vPathEdge.end());
                //Dense hash map?
                vPath.push_back(eNodeID2);
                vPathEdge.push_back(mineID);
                vPath.insert(vPath.end(), vvPathCandidate[pID].begin()+eID2Pos+1, vvPathCandidate[pID].end());
                vPathEdge.insert(vPathEdge.end(), vvPathCandidateEdge[pID].begin()+eID2Pos, vvPathCandidateEdge[pID].end());
                vvmArc.push_back(mmArcTmp);
                vvmPathNodePos.push_back(mmE);
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
		AveSim = AveSim / (kResults.size()*(kResults.size()-1)/2);
		cout << "AveSim: " << AveSim << endl;

		//cout << "max: " << maxSim << endl;
        percentage = countAncestor/(countNonAncestor + countAncestor);
        //cout << "Sim Time:" << SimTime << endl;
        cout << "eKSPCompare countNumber: "<< countNumber << " Pop Path: " << popPath << " Percentage: " << percentage << endl;
    }
    return -1;
}

#include "graph.h"

void Graph::cKSPNew(int ID1, int ID2, int k, vector<int>& kResults, vector<vector<int> >& vkPath)
{
    //Preprocessing
	//Build reverse SP tree
	vector<int> vSPTDistance(nodeNum, INF);
	vector<int> vSPTParent(nodeNum, -1); 
	vector<int> vSPTParentEdge(nodeNum, -1); 
	vector<int> vTmp;
	vector<vector<int> > vSPT(nodeNum, vTmp);  
	vector<int> vSPTCost(nodeNum, INF);
	vector<int> vSPTCostLB(nodeNum, INF);
	vector<int> vCostUB(nodeNum, INF);

	rSPT(ID2, vSPTDistance, vSPTParent, vSPTParentEdge, vSPT);

	//Generating Side Cost
	vector<vector<pair<int, int> > > adjSideDist;  
	vector<vector<pair<int, int> > > adjSideRoad;   
	vector<Edge> vSideEdge;
	vSideEdge.resize(vEdge.size());
	adjSideDist.reserve(nodeNum); 
    boost::thread_group threads;
	threads.add_thread(new boost::thread(&Graph::sideDist, this, boost::ref(vSPTDistance), boost::ref(adjSideDist), boost::ref(adjSideRoad), boost::ref(vSideEdge)));
	label lTmp;
	lTmp.pre = -1;
	lTmp.post = -1;
	lTmp.parent = -1;
	vector<label> vLabel(nodeNum, lTmp); 
	threads.add_thread(new boost::thread(&Graph::intervalLabel, this, ID2, boost::ref(vSPT), boost::ref(vSPTParent), boost::ref(vLabel)));
	threads.join_all();
	
	vector<int> vDistance;
	vector<vector<int> > vvPathCandidate;	//nodes
	vector<vector<int> > vvPathCandidateEdge;//edges
	vector<int> vPathParent;				//Deviated from
	vector<int> vPathParentPos;				//Deviated Pos from Parent
		
	benchmark::heap<2, int, int> qPath(nodeNum*100); 
	
	//Recover the first path
	vector<int> vPath, vPathEdge; 
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
	
	vvPathCandidate.push_back(vPath); 
	vDistance.push_back(d);
	vvPathCandidateEdge.push_back(vPathEdge);
	vPathParent.push_back(-1);
	vPathParentPos.push_back(0);
	
	unordered_set<int> s;
	vector<unordered_set<int> > vPTreeEdge(nodeNum, s); //All the edges in the pseudo tree, used for removeing edges. 

//	cout << vvPathCandidate.size() << endl;
	qPath.update(vvPathCandidate.size()-1, d); 
	int topPathID, topPathDistance;

	vector<PTNode> vPTreeNode; 
	PTNode PTree; 
	unordered_map<int, int> PTRoad;
	unordered_map<int, int> PTChildren;
	PTree.nodeID = ID1; 
	PTree.PTRoad = PTRoad;
	PTree.PTChildren = PTChildren; 
	vPTreeNode.push_back(PTree);

	int oldD = -1;
	int count = 0;
	while(!qPath.empty())
	{
		count++;
		qPath.extract_min(topPathID, topPathDistance);
	
//		cout << count << "\t" << topPathDistance << "\t" << vCost[topPathID] << endl;
//		cout << "top:" << topPathDistance << endl;
		if(topPathDistance < oldD)
			cout << "Error!" << topPathDistance << "\t" << oldD << endl;
		oldD = topPathDistance;
		kResults.push_back(topPathDistance);

		if(count >= k)
			break;
		
		int currentTreeNodeID = 0;
		for(auto& e : vvPathCandidateEdge[topPathID])
		{
			int eID2 = vEdge[e].ID2;
			if(vPTreeNode[currentTreeNodeID].PTChildren.find(eID2) == vPTreeNode[currentTreeNodeID].PTChildren.end())
			{
				PTNode PNodeNew;
				unordered_map<int, int> PTRoadTmp;
				unordered_map<int, int> PTChildrenTmp; 
				PNodeNew.nodeID = eID2;
				PNodeNew.PTRoad = PTRoadTmp;
				PNodeNew.PTChildren = PTChildrenTmp;
				vPTreeNode.push_back(PNodeNew);
				vPTreeNode[currentTreeNodeID].PTRoad[eID2] = e;
				vPTreeNode[currentTreeNodeID].PTChildren[eID2] = vPTreeNode.size()-1; 
			}

			currentTreeNodeID = vPTreeNode[currentTreeNodeID].PTChildren[eID2];  
		}

		vector<vector<pair<int, int> > > adjSideDistTmp = adjSideDist;    
		vector<vector<pair<int, int> > > adjSideRoadTmp = adjSideRoad;    

		//Store the Ignored Edge
		vector<int> vIgnoredNode;
		vIgnoredNode.reserve(nodeNum);
		
		//Sub-path, find the start of the current deviation
		vector<int> vPreviousNode;
		vector<int> vPreviousEdge;

		int i = 0;  
		int subLength = 0; 
		int subCost = 0;
		vPreviousNode.clear();
		vPreviousNode.push_back(ID1);
		vIgnoredNode.push_back(ID1);

		currentTreeNodeID = 0;  
		for(i = 0; i < vPathParentPos[topPathID]; i++)
		{
			int nodeID = vvPathCandidate[topPathID][i];
	//		cout << "nodeID:" << nodeID << endl;
			vPreviousNode.push_back(vvPathCandidate[topPathID][i+1]);
			vPreviousEdge.push_back(vvPathCandidateEdge[topPathID][i]);
			subLength += vEdge[vvPathCandidateEdge[topPathID][i]].length; 
			subCost += vEdge[vvPathCandidateEdge[topPathID][i]].cost;  
			
//			cout << "Current Tree Node:" << vPTreeNode[currentTreeNodeID].nodeID << endl;
			currentTreeNodeID = vPTreeNode[currentTreeNodeID].PTChildren[vvPathCandidate[topPathID][i+1]];

			vIgnoredNode.push_back(vvPathCandidate[topPathID][i]);
		}
	
		//Remove the edges incrementally for each new path
		vIgnoredNode.push_back(vEdge[*(vvPathCandidateEdge[topPathID].begin()+i)].ID1);  
		for(auto ie = vvPathCandidateEdge[topPathID].begin() + i; ie != vvPathCandidateEdge[topPathID].end(); ie++)
		{
			int eID1 = vEdge[*ie].ID1;
			int eID2 = vEdge[*ie].ID2;

			vIgnoredNode.push_back(eID1);
			for(int j = 0; j < (int)adjSideDistTmp[eID1].size();) 
			{
				int eID2Tmp = adjSideDistTmp[eID1][j].first;
				int eRoadID = adjSideRoadTmp[eID1][j].second;
				if(eID2Tmp == eID2)
				{
					adjSideDistTmp[eID1].erase(adjSideDistTmp[eID1].begin() + j);
					adjSideRoadTmp[eID1].erase(adjSideRoadTmp[eID1].begin() + j);  
				}
				else if(vPTreeNode[currentTreeNodeID].PTRoad.find(eID2Tmp) != vPTreeNode[currentTreeNodeID].PTRoad.end())
				{
					adjSideDistTmp[eID1].erase(adjSideDistTmp[eID1].begin() + j);
					adjSideRoadTmp[eID1].erase(adjSideRoadTmp[eID1].begin() + j);  
				}
				else
					j++;
			}

			bool bE = false;
			int dTmp, cTmp;
		
			DijkstraSideCostPruneNew(eID1, ID2, adjSideDistTmp, adjSideRoadTmp, vSPTParent, vSPTParentEdge, vSPTDistance, vLabel, vIgnoredNode, vPath, vPathEdge, bE, subLength, dTmp);
	
			currentTreeNodeID = vPTreeNode[currentTreeNodeID].PTChildren[eID2]; 

			int d2 = subLength; 
			subLength += vEdge[*ie].length; 

			if(bE)
				d2 += dTmp; 
			else if(dTmp != INF)
				for(auto& edgeID : vPathEdge) 
					d2 += vEdge[edgeID].length;
	
			vector<int> vPreviousNodeTmp = vPreviousNode;
			vector<int> vPreviousEdgeTmp = vPreviousEdge;
			
			vPreviousNode.push_back(eID2);
			vPreviousEdge.push_back(*ie);  

			
			if(dTmp == INF)
				continue;
			
			vPreviousNodeTmp.insert(vPreviousNodeTmp.end(), vPath.begin()+1, vPath.end());
			vPreviousEdgeTmp.insert(vPreviousEdgeTmp.end(), vPathEdge.begin(), vPathEdge.end());
	
			vvPathCandidate.push_back(vPreviousNodeTmp);
			vvPathCandidateEdge.push_back(vPreviousEdgeTmp);
			
			vPathParent.push_back(topPathID);
			vPathParentPos.push_back(ie - vvPathCandidateEdge[topPathID].begin());
		
			qPath.update(vvPathCandidate.size()-1, d2);
			vDistance.push_back(d2);
		}	
	}
}

int Graph::DijkstraSideCostPruneNew(int ID1, int ID2, vector<vector<pair<int, int> > >& adjSideDist, vector<vector<pair<int, int> > >& adjSideRoad, vector<int>& vSPTParent, vector<int>& vSPTParentEdge, vector<int>& vSPTDistance, vector<label>& vLabel, vector<int>& vIgnoredNode, vector<int>& vPath, vector<int>& vPathEdge, bool& bE, int& subLength,  int& dTmp)      
{
	benchmark::heap<2, int, int> queue(nodeNum);
	queue.update(ID1, 0);

	vector<int> vDistance(nodeNum, INF);
	vector<int> vRealDist(nodeNum, INF);
	vector<int> vPrevious(nodeNum, -1);
	vector<int> vPreviousEdge(nodeNum, -1);
	vector<bool> vbVisited(nodeNum, false);
	int topNodeID, neighborNodeID, neighborLength, neighborRoadID;
	vector<pair<int, int> >::iterator ivp;
	
	for(auto& v : vIgnoredNode) 
		vbVisited[v] = true;

	vDistance[ID1] = 0;
	vRealDist[ID1] = subLength;

	while(!queue.empty())
	{
		int topDistance; 
		queue.extract_min(topNodeID, topDistance); 
		vbVisited[topNodeID] = true;

		int topRealDist = vRealDist[topNodeID];  

		if(topNodeID == ID2)
			break;

		//Early Termination
		bool b = true;
		if(topNodeID != ID1)
		{
			for(auto& iNode: vIgnoredNode)
			{
				if(!(vLabel[topNodeID].pre < vLabel[iNode].pre || vLabel[topNodeID].post > vLabel[iNode].post)) 
				{
					b = false;
					break;
				}
			}
		}
		else
			b = false;

		if(b)
		{
			bE = true;
			vPath.clear();
			vPathEdge.clear();
			vPath.push_back(topNodeID);
			int p = vPrevious[topNodeID];
			int e = vPreviousEdge[topNodeID];
			int d = 0;

			while(p != -1)
			{
				vPath.push_back(p);
				vPathEdge.push_back(e);
				d += vEdge[e].length; 
				e = vPreviousEdge[p];
				p = vPrevious[p];
			}

			reverse(vPath.begin(), vPath.end());
			reverse(vPathEdge.begin(), vPathEdge.end());
				
			p = topNodeID;
			while(vSPTParent[p] != -1)
			{
				vPathEdge.push_back(vSPTParentEdge[p]);
				p = vSPTParent[p];
				vPath.push_back(p);
			}
			dTmp = d + vSPTDistance[topNodeID];
			return  d + vSPTDistance[topNodeID];
		}

		for(int i = 0; i < (int)adjSideDist[topNodeID].size(); i++) 
		{
			neighborNodeID = adjSideDist[topNodeID][i].first;
			neighborLength = adjSideDist[topNodeID][i].second; 
			neighborRoadID = adjSideRoad[topNodeID][i].second;
			int d = vDistance[topNodeID] + neighborLength; 
			if(!vbVisited[neighborNodeID])
			{
				if(vDistance[neighborNodeID] > d)
				{
					vDistance[neighborNodeID] = d; 
					vRealDist[neighborNodeID] = vRealDist[topNodeID] + vEdge[neighborRoadID].length;
					queue.update(neighborNodeID, d);
					vPrevious[neighborNodeID] = topNodeID;
					vPreviousEdge[neighborNodeID] = neighborRoadID;
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
//		cout << p << "\t" << e << endl;
		vPath.push_back(p);
		vPathEdge.push_back(e);
		e = vPreviousEdge[p];
		p = vPrevious[p];
	}

	reverse(vPath.begin(), vPath.end());
	reverse(vPathEdge.begin(), vPathEdge.end());

	bE = false; 
	dTmp = vDistance[ID2];
	return vDistance[ID2];
}

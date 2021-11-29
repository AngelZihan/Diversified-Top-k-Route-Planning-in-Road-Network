#include "graph.h"
   
int main()
{
	string filename = "USA-road-d.NY.gr";
    Graph g = Graph(filename);
    cout << "Graph loading finish" << endl;
    srand (time(NULL));
    std::chrono::high_resolution_clock::time_point t1;
    std::chrono::high_resolution_clock::time_point t2;
    std::chrono::duration<double> time_span;
    vector<int> vPath, vPathEdge;

    vector<int> ID1List;
    vector<int> ID2List;
    string queryFilename = "USA-NY-Q2.txt";
    ifstream inGraph(queryFilename);
    if(!inGraph)
        cout << "Cannot open Map " << queryFilename << endl;
    cout << "Reading " << queryFilename << endl;
    int pID1, pID2;
    string line;
    getline(inGraph,line);
    while(!inGraph.eof())
    {
        vector<string> vs;
        boost::split(vs,line,boost::is_any_of(" "),boost::token_compress_on);
        pID1 = stoi(vs[0]) - 1;
        pID2 = stoi(vs[1]) - 1;
        ID1List.push_back(pID1);
        ID2List.push_back(pID2);
        getline(inGraph, line);
    }
    inGraph.close();

	double sumTime = 0;
    int sumCount = 0;
    int sumPop = 0;
	int sumLength = 0;
	double sumPercentage = 0;
	float sumSimTime = 0;
    double meanTime;
    int meanCount;
    int meanPop;
	int meanLength;
	double meanPercentage;
	float meanSimTime;

    vector<double> cTKSPDTime;
    vector<int> cTKSPDCount;
    vector<int> cTKSPDPop;
    vector<int> cTKSPDAverageLength;

    vector<double> eKSPTime;
    vector<int> eKSPCount;
    vector<int> eKSPPop;
	vector<int> eKSPAverageLength;
	vector<float> eKSPSimTime;

    vector<double> eKSPCompareTime;
    vector<int> eKSPCompareCount;
    vector<int> eKSPComparePop;
	vector<int> eKSPCompareAverageLength;
	vector<double> eKSPComparePercentage;
	vector<float> eKSPCompareSimTime;

    vector<double> eKSPPruneTime;
    vector<int> eKSPPruneCount;
    vector<int> eKSPPrunePop;
	vector<int> eKSPPruneAverageLength;
	vector<double> eKSPPrunePercentage;

    int ID1, ID2;
    for (int i = 0; i < ID1List.size(); ++i) {
        ID1 = ID1List[i];
        ID2 = ID2List[i];
		cout << "ID1:" << ID1 << "\tID2:" << ID2 << endl;
        int k = 5;
		double t = 0.9; 
		cout << "k: " << k << " t: " << t << endl;
        vector<int> kResults;
        int countNumber = 0;
        int popPath = 0;
		vector<float> sim;
		float percentage;
		float SimTime;
        kResults.reserve(k);
        vector<vector<int> >vkPath;
        g.FindRepeatedPath(vkPath);

        kResults.clear();
        vkPath.clear();
        t1 = std::chrono::high_resolution_clock::now();
        g.cTKSPD(ID1, ID2, k, kResults, t, countNumber, popPath);
        t2 = std::chrono::high_resolution_clock::now();
        time_span = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1);
        if(popPath != -1){
            cTKSPDTime.push_back(time_span.count());
            cTKSPDCount.push_back(countNumber);
            cTKSPDPop.push_back(popPath);
            cout << "cTKSPD Time:" << time_span.count() << endl;
            int cTKSPDPathAverageLength = 0;
            for(auto& d : kResults){
                cout << d << "\t";
                cTKSPDPathAverageLength += d;
            }
            cTKSPDAverageLength.push_back( cTKSPDPathAverageLength / kResults.size());
            cout << endl;
            cout << "Path Average Length: " << cTKSPDPathAverageLength / kResults.size() << endl;
            cout << endl;
            cout << endl;
        }


		kResults.clear();
        vkPath.clear();
        t1 = std::chrono::high_resolution_clock::now();
        g.eKSPNew(ID1, ID2, k, kResults, vkPath, t, countNumber, popPath, sim, SimTime);
        t2 = std::chrono::high_resolution_clock::now();
        time_span = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1);
		if(popPath != -1){
			eKSPTime.push_back(time_span.count());
			eKSPCount.push_back(countNumber);
			eKSPPop.push_back(popPath);
			eKSPSimTime.push_back(SimTime);
			cout << "eKSP Time:" << time_span.count() << endl;
			int eKSPPathAverageLength = 0;
			for(auto& d : kResults){
				cout << d << "\t";
				eKSPPathAverageLength += d; 
			}
			eKSPAverageLength.push_back(eKSPPathAverageLength / kResults.size());
			cout << endl;
			cout << "Path Average Length: " << eKSPPathAverageLength / kResults.size() << endl;
			cout << endl;
			cout << endl;
		}


		kResults.clear(); 
        vkPath.clear();
		sim.clear();
        t1 = std::chrono::high_resolution_clock::now();
        g.eKSPCompare(ID1, ID2, k, kResults, vkPath, t, countNumber, popPath, percentage, SimTime);
        t2 = std::chrono::high_resolution_clock::now();
        time_span = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1);
		if(popPath != -1){
			eKSPCompareTime.push_back(time_span.count());
			eKSPCompareCount.push_back(countNumber);
			eKSPComparePop.push_back(popPath);
			eKSPComparePercentage.push_back(percentage);
			eKSPCompareSimTime.push_back(SimTime);
			cout << "eKSPCompare Time:" << time_span.count() << endl;
			int eKSPComparePathAverageLength = 0;
			for(auto& d : kResults){
				cout << d << "\t";
				eKSPComparePathAverageLength += d; 
			}
			eKSPCompareAverageLength.push_back(eKSPComparePathAverageLength / kResults.size());
			cout << endl;
			cout << "Path Average Length: " << eKSPComparePathAverageLength / kResults.size() << endl;
			cout << endl;
			cout << endl;
		}

		
	    kResults.clear();
        vkPath.clear();
        t1 = std::chrono::high_resolution_clock::now();
        g.eKSPPrune(ID1, ID2, k, kResults, vkPath, t, countNumber, popPath);
        t2 = std::chrono::high_resolution_clock::now();
        time_span = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1);
		if(popPath != -1){
			eKSPPruneTime.push_back(time_span.count());
			eKSPPruneCount.push_back(countNumber);
			eKSPPrunePop.push_back(popPath);
			//eKSPPrunePercentage.push_back(percentage);
			cout << "eKSPPrune Time:" << time_span.count() << endl;
			int eKSPPrunePathAverageLength = 0;
			for(auto& d : kResults){
				cout << d << "\t";
				eKSPPrunePathAverageLength += d; 
			}
			eKSPPruneAverageLength.push_back(eKSPPrunePathAverageLength / kResults.size());
			cout << endl;
			cout << "Path Average Length: " << eKSPPrunePathAverageLength / kResults.size() << endl;
			cout << endl;
			cout << endl;
		}

    }

	sumTime = 0;
    sumCount = 0;
    sumPop = 0;
	sumLength = 0;
    for (int i = 0; i < cTKSPDTime.size(); i++)
    {
        sumTime += cTKSPDTime[i];
        sumCount += cTKSPDCount[i];
        sumPop += cTKSPDPop[i];
		sumLength += cTKSPDAverageLength[i];
    }
    meanTime = sumTime / cTKSPDTime.size();
    meanCount = sumCount / cTKSPDCount.size();
    meanPop = sumPop / cTKSPDPop.size();
	meanLength = sumLength / cTKSPDAverageLength.size();
    cout  << "cTKSPD Average Time: " << meanTime << "\tcTKSPD Average Count:" << meanCount << "\tcTKSPD Average Pop: " << meanPop << " \tcTKSPD Average Length: " << meanLength << endl;


    sumTime = 0;
    sumCount = 0;
    sumPop = 0;
	sumLength = 0;
	sumSimTime = 0;
    for (int i = 0; i < eKSPTime.size(); i++)
    {
        sumTime += eKSPTime[i];
        sumCount += eKSPCount[i];
        sumPop += eKSPPop[i];
		sumLength += eKSPAverageLength[i];
		sumSimTime += eKSPSimTime[i];
    }
    meanTime = sumTime / eKSPTime.size();
    meanCount = sumCount / eKSPCount.size();
    meanPop = sumPop / eKSPPop.size();
	meanLength = sumLength / eKSPAverageLength.size();
	meanSimTime = sumSimTime / eKSPSimTime.size();
	cout << "Pair Size:" << eKSPTime.size() << endl;
    cout  <<"eKSP Average Time: " << meanTime << "\teKSP Average Count: " << meanCount << "\teKSP Average Pop: " << meanPop << " \teKSP Average Length: " << meanLength << " \t eKSP SimTime: " << meanSimTime << endl;


    sumTime = 0;
    sumCount = 0;
    sumPop = 0;
	sumLength = 0;
	sumPercentage = 0;
	sumSimTime = 0;
    for (int i = 0; i < eKSPCompareTime.size(); i++)
    {
        sumTime += eKSPCompareTime[i];
        sumCount += eKSPCompareCount[i];
        sumPop += eKSPComparePop[i];
		sumLength += eKSPCompareAverageLength[i];
		sumPercentage += eKSPComparePercentage[i];
		sumSimTime += eKSPCompareSimTime[i];
    }
    meanTime = sumTime / eKSPCompareTime.size();
    meanCount = sumCount / eKSPCompareCount.size();
    meanPop = sumPop / eKSPComparePop.size();
	meanLength = sumLength / eKSPCompareAverageLength.size();
	meanPercentage = sumPercentage / eKSPComparePercentage.size();
	meanSimTime = sumSimTime / eKSPCompareSimTime.size();
	cout << "Pair Size:" << eKSPCompareTime.size() << endl;
    cout  <<"eKSPCompare Average Time: " << meanTime << "\teKSPCompare Average Count: " << meanCount << "\teKSPCompare Average Pop: " << meanPop << " \teKSPCompare Average Length: " << meanLength<< " \teKSPCompare Average Percentage: " << meanPercentage  << " \teKSPCompare SimTime: " << meanSimTime << endl;


    sumTime = 0;
    sumCount = 0;
    sumPop = 0;
	sumLength = 0;
    for (int i = 0; i < eKSPPruneTime.size(); i++)
    {
        sumTime += eKSPPruneTime[i];
        sumCount += eKSPPruneCount[i];
        sumPop += eKSPPrunePop[i];
		sumLength += eKSPPruneAverageLength[i];
    }
    meanTime = sumTime / eKSPPruneTime.size();
    meanCount = sumCount / eKSPPruneCount.size();
    meanPop = sumPop / eKSPPrunePop.size();
	meanLength = sumLength / eKSPPruneAverageLength.size();
	cout << "Pair Size:" << eKSPPruneTime.size() << endl;
    cout  <<"eKSPPrune Average Time: " << meanTime << "\teKSPPrune Average Count: " << meanCount << "\teKSPPrune Average Pop: " << meanPop << " \teKSPPrune Average Length: " << meanLength<< endl;

    return 0;
}

void Graph::FindRepeatedPath(vector<vector<int> >& vvPath)
{
    for(int i = 0; i < (int)vvPath.size()-1; i++)
    {
        vector<int> vSame;
        for(int j = i + 1; j < (int)vvPath.size(); j++)
        {
            if(vvPath[i].size() != vvPath[j].size())
                continue;

            bool bSame = true;
            for(int k = 0; k < vvPath[i].size(); k++)
            {
                if(vvPath[i][k] != vvPath[j][k])
                {
                    bSame = false;
                    break;
                }
            }

            if(bSame)
                vSame.push_back(j);
        }

        if(!vSame.empty())
        {
            cout << "Same path of " << i << ":" << endl;
            for(auto& sID : vSame)
                cout << sID << "\t";
            cout << endl;
        }
    }
}

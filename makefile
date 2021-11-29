CXX=g++ -std=c++17
OPT=-O3

DKSP: main.o graph.o eKSP.o cTKSPD.o eKSPCompare.o eKSPPrune.o -lpthread -lboost_system -lboost_thread
	$(CXX) -g -o DKSP main.o graph.o eKSP.o cTKSPD.o eKSPCompare.o eKSPPrune.o -lpthread -lboost_system -lboost_thread

graph.o:graph.cpp
	$(CXX) -g -c $(OPT) graph.cpp
main.o:main.cpp
	$(CXX) -g -c $(OPT) main.cpp
eKSP.o:eKSP.cpp
	$(CXX) -g -c $(OPT) eKSP.cpp
cTKSPD.o: cTKSPD.cpp
	$(CXX) -g -c $(OPT) cTKSPD.cpp
eKSPCompare.o: eKSPCompare.cpp
	$(CXX) -g -c $(OPT) eKSPCompare.cpp
eKSPPrune.o: eKSPPrune.cpp
	$(CXX) -g -c $(OPT) eKSPPrune.cpp
	
clean:
	rm *.o
	rm DKSP

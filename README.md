# Diversified-Top-k-Route-Planning-in-Road-Network

There are four algorithms in this repository:

| Algorithm | Description|
| ------ | ------ |
| cTKSPD | Implementation of the 'Finding Top-k Shortest Paths with Diversity' algorithm (TKDE 2017)|
| eKSP | Diversified k-Path Enumeration algorithm |
| eKSPCompare | Improve eKSP algorithm by using LCA and inheritance relationship to compare the similarity|
| eKSPPrune | Improve eKSPCompare algorithm by pruning useless deviation edge |

## Running

Two steps to run these algorithms:

The first step is to compile: 

```sh
$ make
```

Then, use the code to run:

```sh
$ ./DKSP
```

## Change Parameter
| Parameter | File | How to Change |
| ------ | ------ | ------ |
| Road File | main.cpp | Change the 'filename' address |
| Query File | main.cpp | Change the 'queryFilename' address |
| Algorithm | main.cpp | Uncomment the algorithm |
| Path Number | main.cpp | Change the 'k' value |
| Similarity Threshold | main.cpp | Change the 't' value |
| Similarity method | Algorithm File | Uncomment the similarity method and comment all others|

## Running Result

The running result shows the similarity of each path, path ID, running time, generated path number and the length of each path.

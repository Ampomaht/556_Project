// ECE556 - Copyright 2013 University of Wisconsin-Madison.  All Rights Reserved.

#include "ece556.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cstring>
#include <stdlib.h>
#include <vector>

using namespace std;

/*
	ADDIDITONAL FUCTIONS
*/
std::vector<string> &split(const string &s, char delim, vector<string> &elems);
std::vector<string> split(const string &s, char delim);

int readBenchmark(const char *fileName, routingInst *rst)
{
	//Blockage updates
	int blockages;
	int new_capacity;

	cout << "Attempt to read from file" << endl;
	ifstream myfile;
	myfile.open(fileName);
	string line;
	std::vector<string> buffer;
	if (!myfile.good()) {
		cout << "Error: File could not open";
		return 0;
	}

	//Parse Grid Dimensions
	cout << "Parsing data, creating routing instance" << endl;
	getline(myfile, line);
	buffer = split(line, ' ');
	rst->gx = atoi(buffer.at(1).c_str()); 
	rst->gy = atoi(buffer.at(2).c_str()); 

	// Initialize edge array
	int totalNumEdge = rst->gy*(rst->gx - 1) + (rst->gy - 1)*rst->gx;
	rst->edges = new edge[totalNumEdge];

	//Get Capacity
	getline(myfile, line);
	buffer = split(line, ' ');
	rst->cap = atoi(buffer.at(1).c_str()); 

	// Initialize edge points
	// Across edge first
	for (int i = 0; i < totalNumEdge; i++) {
		rst->edges[i].cap = rst->cap;
		rst->edges[i].util = 0;
	}

		


	// Initialize edge points
	// vertical first
	int gx = 0, gy = 0, index = 0;
	while(gy < rst->gy) {
		if (gx != rst->gx - 1) {
			rst->edges[index].p1.x = gx;
			rst->edges[index].p1.y = gy;
			gx++;
			rst->edges[index].p2.x = gx;
			rst->edges[index].p2.y = gy;
					cout << rst->edges[index].p1.x << ',' << rst->edges[index].p1.y << ' ';
		cout << rst->edges[index].p2.x << ',' << rst->edges[index].p2.y << endl;
			index++;
		} 
		else {
			gx = 0;
			gy++;
		}

		
	}

			// Initialize edge points
	// horizontal edges
	gx = 0, gy = 0;
	while(gx < rst->gx) {
		if (gy != rst->gy - 1) {
			rst->edges[index].p1.x = gx;
			rst->edges[index].p1.y = gy;

			gy++;
			rst->edges[index].p2.x = gx;
			rst->edges[index].p2.y = gy;
					cout << rst->edges[index].p1.x << ',' << rst->edges[index].p1.y << ' ';
		cout << rst->edges[index].p2.x << ',' << rst->edges[index].p2.y << endl;
			index++;
		} 
		else {
			gy = 0;
			gx++;
		}

	}



	//Get the Num Nets
	getline(myfile, line);
	buffer = split(line, ' ');
	rst->numNets = atoi(buffer.at(2).c_str()); 

	//Get Nets & Net Data
	rst->nets = new net[rst->numNets];
	for (int i = 0; i < rst->numNets; i++) {
		getline(myfile, line);			// this line is the net's name and number of pins
		buffer = split(line, ' ');
		rst->nets[i].id = i;
		rst->nets[i].numPins = atoi(buffer.at(1).c_str()); 
		rst->nets[i].pins = new point[rst->nets[i].numPins];
		for (int j = 0; j < rst->nets[i].numPins; j++){	// edges
			getline(myfile,line);
			buffer = split(line, ' ');
			rst->nets[i].pins[j].x = atoi(buffer.at(0).c_str());
			rst->nets[i].pins[j].y = atoi(buffer.at(1).c_str());
		}
	}

	////Get Blockage Data
	getline(myfile, line);
	blockages = atoi(line.c_str());
	rst->bEgdes = new segment[blockages];
	for (int i = 0; i < blockages; i++) {
		getline(myfile, line);
		buffer = split(line, ' ');
		rst->bEgdes[i].p1.x = atoi(buffer.at(0).c_str());
		rst->bEgdes[i].p1.y = atoi(buffer.at(1).c_str());
		rst->bEgdes[i].p2.x = atoi(buffer.at(2).c_str());
		rst->bEgdes[i].p2.y = atoi(buffer.at(3).c_str());
		new_capacity = atoi((buffer.at(4).c_str()));
	}
	cout << "Routing instance created with no errors!\n\n" << 
		"Grid summary:\nGrid dimension is " << rst->gx << " by " << rst->gy 
		<< "\nCapacity for normal edge is " << rst->cap 
		<< "\nTotal number of nets is " << rst->numNets
		<< "\nTotal number of blockages is " << blockages << endl;


  return 1;
}

std::vector<string> &split(const string &s, char delim, vector<string> &elems) 
{
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

std::vector<string> split(const string &s, char delim) 
{
    vector<string> elems;
    split(s, delim, elems);
    return elems;
}

int solveRouting(routingInst *rst){
  

  return 1;
}

int writeOutput(const char *outRouteFile, routingInst *rst){
  /*********** TO BE FILLED BY YOU **********/

  return 1;
}


int release(routingInst *rst){
  /*********** TO BE FILLED BY YOU **********/

  return 1;
}
  


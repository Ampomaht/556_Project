// ECE556 - Copyright 2013 University of Wisconsin-Madison.  All Rights Reserved.

#include "ece556.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cstring>
#include <stdlib.h>
#include <vector>
#include <limits.h>

using namespace std;

/*
	ADDIDITONAL FUCTIONS
*/
std::vector<string> &split(const string &s, char delim, vector<string> &elems);
std::vector<string> split(const string &s, char delim);
int initialRoute(routingInst *rst);
int reroute(routingInst *rst);

/* Hash map data structure to hold */
static map<string, edge*> edges;     /* hashmap containing string key to unique edge value */

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
	buffer = split(line, '\t');
	buffer = split(buffer.at(1), ' ');
	rst->gx = atoi(buffer.at(0).c_str()); 
	rst->gy = atoi(buffer.at(1).c_str()); 

	// Initialize edge array
	int totalNumEdge = rst->gy*(rst->gx - 1) + (rst->gy - 1)*rst->gx;
	//rst->edges = new edge[totalNumEdge];

	//Get Capacity
	getline(myfile, line);
	buffer = split(line, '\t');
	rst->cap = atoi(buffer.at(1).c_str()); 

	cout << "Initialing edge map data structure. . ." << endl;
	// Initialize edge points
	// vertical first
	int gx = 0, gy = 0;
	int i = 0;
	while(gy < rst->gy) {
		//cout << "edge no." << i << endl;
		edge *e = new edge;
		string key;
		if (gx != rst->gx - 1) {
			e->p1.x = gx;
			e->p1.y = gy;
			gx++;
			e->p2.x = gx;
			e->p2.y = gy;
			e->cap = rst->cap;
			e->util = 0;

			key = static_cast<ostringstream*>(
				&(ostringstream() << e->p1.x << e->p1.y << e->p2.x << e->p2.y) )->str();
			edges[key] = e;
			key = static_cast<ostringstream*>(
				&(ostringstream() << e->p2.x << e->p2.y << e->p1.x << e->p1.y) )->str();
			edges[key] = e;

			//cout << edges[key]->p1.x << ',' << edges[key]->p1.y << ' ';
			//cout << edges[key]->p2.x << ',' << edges[key]->p2.y << endl;
		} 
		else {
			gx = 0;
			gy++;
		}	
		i++;
	}

	// Initialize edge points
	// horizontal edges
	gx = 0, gy = 0;
	while(gx < rst->gx) {
		edge *e = new edge;
		string key;
		if (gy != rst->gy - 1) {
			e->p1.x = gx;
			e->p1.y = gy;
			gy++;
			e->p2.x = gx;
			e->p2.y = gy;
		
			key = static_cast<ostringstream*>(
				&(ostringstream() << e->p1.x << e->p1.y << e->p2.x << e->p2.y) )->str();
			edges[key] = e;
			key = static_cast<ostringstream*>(
				&(ostringstream() << e->p2.x << e->p2.y << e->p1.x << e->p1.y) )->str();
			edges[key] = e;
			//cout << edges[key]->p1.x << ',' << edges[key]->p1.y << ' ';
			//cout << edges[key]->p2.x << ',' << edges[key]->p2.y << endl;
		} 
		else {
			gy = 0;
			gx++;
		}

	}

	cout << "DONE with edge mapping" << endl;

	//Get the Num Nets
	getline(myfile, line);
	buffer = split(line, ' ');
	rst->numNets = atoi(buffer.at(2).c_str()); 

	//Get Nets & Net Data
	rst->nets = new net[rst->numNets];
	
	cout << "Initializing nets and pins" << endl;

	for (int i = 0; i < rst->numNets; i++) {
		getline(myfile, line);			// this line is the net's name and number of pins
		buffer = split(line, ' ');
		rst->nets[i].id = i;
		rst->nets[i].numPins = atoi(buffer.at(1).c_str()); 
		rst->nets[i].pins = new pin[rst->nets[i].numPins];
		
		for (int j = 0; j < rst->nets[i].numPins; j++){	// pins
			//cout << "Init net " << i << " pin " << j <<  endl;
			rst->nets[i].pins[j].isConnected = false;
			getline(myfile,line);
			buffer = split(line, '\t');
			rst->nets[i].pins[j].loc.x = atoi(buffer.at(0).c_str());
			rst->nets[i].pins[j].loc.y = atoi(buffer.at(1).c_str());  // SPACE PROBLEM!!!!!
		}
	}

	
	cout << "DONE with net mapping" << endl;

	//Get Blockage Data
	getline(myfile, line);
	blockages = atoi(line.c_str());
	rst->bEgdes = new segment[blockages];
	vector<string> buffer1, buffer2;
	for (int i = 0; i < blockages; i++) {
		getline(myfile, line);
		buffer = split(line, '\t');
		buffer1 = split(buffer.at(0), ' ');
		buffer2 = split(buffer.at(1), ' ');
		rst->bEgdes[i].p1.x = atoi(buffer1.at(0).c_str());
		rst->bEgdes[i].p1.y = atoi(buffer1.at(1).c_str());
		rst->bEgdes[i].p2.x = atoi(buffer2.at(0).c_str());
		rst->bEgdes[i].p2.y = atoi(buffer2.at(1).c_str());
		new_capacity = atoi((buffer2.at(2).c_str()));
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

int getXDist(point a, point b)  
{ 
  int dist = a.x - b.x;
  return -dist;

}

int getYDist(point a, point b)  
{ 
  int dist = a.y - b.y;
  return -dist;
}

int getDist(point a, point b)  
{ 
	int distX = a.x - b.x;
  int distY = a.y - b.y;
  return abs(distX) + abs(distY);
}

/**
* Perform initial routing 
*/
int initialRoute(routingInst *rst) 
{
	cout << "Perform initial routing. . . Please wait this can take a few mins" << endl;
	vector<segment> segments;

	for (int i = 0; i < rst->numNets; i++) {

		// Initialize stuff
		rst->nets[i].numCRoutes = 1; // TODO
		route *r = new route;
		r->numSegs = 0;
		r->weight = 0;

		segments.clear();

		for (int j = 0; j < rst->nets[i].numPins; j++) {
			
			// if not connected
				
			if (!rst->nets[i].pins[j].isConnected) {
				rst->nets[i].pins[j].isConnected = true;
				pin thisPin = rst->nets[i].pins[j];
				pin nextPin ;

				// find closest point to route
				int k, temp, minDist = INT_MAX, minIndex = j+1 ; // default to next one in line 
				if (rst->nets[i].numPins > 2) {
					for (k = 0; k < rst->nets[i].numPins; k++) {
						if (k != j) {
							temp = getDist(thisPin.loc, rst->nets[i].pins[k].loc);
							if (temp < minDist && rst->nets[i].pins[k].isConnected) {
								minDist = temp;
								minIndex = k;
							} 
							else if (j == 0 && temp < minDist) {
								minDist = temp;
								minIndex = k;
							}
						}
					}
				} 

				nextPin = rst->nets[i].pins[minIndex];

				// Route horizontally				
				int distX = getXDist(thisPin.loc, nextPin.loc);
				segment *seg = new segment;
				seg->p1 = thisPin.loc;				//define starting point
				point p;
				p.x = nextPin.loc.x;
				p.y = thisPin.loc.y;
				seg->p2 = p;
				int index = 0, weight = 0;
				seg->edges = new edge[abs(distX)];
				seg->weight = 0;
				seg->numEdges = 0;

				point currPoint;			// define current starting point of the edge we're working 
				currPoint = thisPin.loc;
				while (distX != 0) {
					string key;

					// move right
					if (distX > 0) {
						distX = distX - 1;
						key = static_cast<ostringstream*>(
						&(ostringstream() << currPoint.x << currPoint.y << currPoint.x+1 << currPoint.y) )->str();
						currPoint.x = currPoint.x+1;
					}
					// move left
					else {
						distX = distX + 1;	
						key = static_cast<ostringstream*>(
						&(ostringstream() << currPoint.x << currPoint.y << currPoint.x-1 << currPoint.y) )->str();
						currPoint.x = currPoint.x-1;
					}
					
					edges[key]->util++;
					edge *e = edges[key];
					seg->edges[index] = *e;
					seg->numEdges++;	
					if (edges[key]->cap == 0) {
						weight = INT_MAX;
					}
					else {
						weight = edges[key]->util / edges[key]->cap;
					}
					
					seg->weight += weight;

					index++;
				}

				// Check if the end of horizontal segment is the last pin we're looking for
				if ( currPoint.x == rst->nets[i].pins[minIndex].loc.x && 
					currPoint.y == rst->nets[i].pins[minIndex].loc.y) {
						rst->nets[i].pins[minIndex].isConnected = true;
				} 

				r->numSegs++;
				r->weight += seg->weight;
				segments.push_back(*seg);

				// route vertically
				int distY = getYDist(thisPin.loc, nextPin.loc);
				seg = new segment;
				p.x = nextPin.loc.x;
				p.y = thisPin.loc.y;
				seg->p1 = p;				//define starting point
				seg->p2 = nextPin.loc;
				index = 0, weight = 0;
				seg->edges = new edge[abs(distY)];
				seg->weight = 0;
				seg->numEdges = 0;
				while (distY != 0) {
					string key;
					// down
					if (distY > 0) {
						distY = distY - 1;
						key = static_cast<ostringstream*>(
						&(ostringstream() << currPoint.x << currPoint.y << currPoint.x << currPoint.y+1) )->str();
						currPoint.y = currPoint.y+1;
					}
					// up
					else {
						distY = distY + 1;	
						key = static_cast<ostringstream*>(
						&(ostringstream() << currPoint.x << currPoint.y << currPoint.x << currPoint.y-1) )->str();
						currPoint.y = currPoint.y-1;
					}
					
						edges[key]->util++;
						edge *e = edges[key];
						seg->edges[index] = *e;
						seg->numEdges++;
						weight = edges[key]->util / edges[key]->cap;
						seg->weight += weight;
					index++;
				}

				// Check if the end of vertical segment is the last pin we're looking for
				if ( currPoint.x == rst->nets[i].pins[minIndex].loc.x && 
					currPoint.y == rst->nets[i].pins[minIndex].loc.y) {
						rst->nets[i].pins[minIndex].isConnected = true;
				} 

				r->numSegs++;
				r->weight += seg->weight;
				segments.push_back(*seg);

				/* DONE INITIAL ROUTING */
			}

		}
					
		// set up route var
		int totalSeg = segments.size();
		r->segments = new segment[totalSeg];
		for (int i = 0; i < totalSeg; i++) {
			r->segments[i] = segments.at(i);
		}

		rst->nets[i].croutes = r;
	}

	cout << "Done with initial routing" << endl;
	return 1;
}

/**
* Ripup and reroute process
*/
int reroute(routingInst *rst) 
{

	return 1;
}

/**
* Solve routing 
**/
int solveRouting(routingInst *rst)
{
	int status = initialRoute(rst);	// Initial Routing 
	int iter;		// How many iterations to perform
	for (iter = 1; iter > 0; iter--) {
		//reroute(rst);
	}
	
	return 1;
}



int writeOutput(const char *outRouteFile, routingInst *rst)
{
  cout << "Writing Output to file " << "outRouteFile" << endl;
  ofstream out;
  out.open (outRouteFile);
	for (int i = 0; i < rst->numNets; i++) {
		out << "n" << rst->nets[i].id << endl;
		route *croute = rst->nets[i].croutes;
		for (int j = 0; j < croute->numSegs; j++) {
			out << "(" << croute->segments[j].p1.x << "," << croute->segments[j].p1.y << ")-(" 
				<< croute->segments[j].p2.x << "," << croute->segments[j].p2.y << ")" << endl;
		}
		out << "!" << endl;
	}

  return 1;
}


int release(routingInst *rst){
  /*********** TO BE FILLED BY YOU **********/

  return 1;
}
  


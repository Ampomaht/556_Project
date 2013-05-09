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
#include <algorithm>

using namespace std;

/*
	ADDIDITONAL FUCTIONS
*/
std::vector<string> &split(const string &s, char delim, vector<string> &elems);
std::vector<string> split(const string &s, char delim);
int initialRoute(routingInst *rst);
int reroute(routingInst *rst);

vector<segment*> findPathUsingAStar(point s, point t, routingInst *rst, int netNum);
vector<point> findAdjacentVertices(point, int max_x, int max_y);
int getDist(point a, point b);
vector<segment*> retrace(point, point, routingInst *rst, int netNum);

// Queue functions
void enqueue(pair<point, double>);
void dequeue();
pair<point, double> extractMin();

/*
 ADDITIONAL OBJECTS
*/
vector<net> nets;
vector<pair<point, double>> pq;				// priority queue used in A* <point, score(point)> 
vector< point > rp;		// storing parents for retracing paths
const int EDGE_BLOCK_MULTIPLIER = 2;		// used to multiply the edge util to get weight if cap is 0

/* Hash map data structure to hold */
static map<string, edge*> edges;     /* hashmap containing string key to unique edge value */

struct FindFirst {
    FindFirst(point p) : toFind(p) { }
    point toFind;
    bool operator() 
        ( const std::pair<point, double > &pair ) {
            return (pair.first.x==toFind.x) && (pair.first.y==toFind.y);
    }
};


///////////////////////////////////////////////////////////////////////////////////////////////////////
/**
* Enqueue an item pair of vertex point and its score
*/
void enqueue(pair<point, double> v) 
{
	vector<pair<point, double>>::iterator it_PQ;
	for (it_PQ = pq.begin(); it_PQ != pq.end(); it_PQ++) {
		if (v.second <= (*it_PQ).second) {
			pq.insert(it_PQ, v);
			return;
		} 
	}
	if (it_PQ == pq.end()) {
		pq.insert(it_PQ, v); 
	}
}

/**
* Dequeue an element from the queue
*/
void dequeue() 
{
	pq.erase(pq.begin());
}

/**
* Get the first element of the queue but it's not out of the queue
*/
pair<point, double> extractMin() 
{
	return pq.front();
}

/**
* Returns a vector of adjacent vertices given a point
*/ 
vector<point> findAdjacentVertices(point p, int max_x, int max_y) 
{
	vector<point> p_adj;

	// check left adj is null
	if (p.x-1 >= 0) {
		point v;
		v.x = p.x-1;
		v.y = p.y;
		p_adj.push_back(v);
	}
	// check top adj is null
	if (p.y-1 >= 0) {
		point v;
		v.x = p.x;
		v.y = p.y-1;
		p_adj.push_back(v);
	}
	// check right adj is null
	if (p.x+1 <= max_x-1) {
		point v;
		v.x = p.x+1;
		v.y = p.y;
		p_adj.push_back(v);
	}
	// check bottom adj is null
	if (p.y+1 <= max_y-1) {
		point v;
		v.x = p.x;
		v.y = p.y+1;
		p_adj.push_back(v);
	}
	return p_adj;
}

/**
* Retrace steps back to src vertex and update edges' utils and weights
*/
vector<segment*> retrace(point dst, point src, routingInst *rst, int netNum)
{
	point cur;
	vector<segment*> path;
	vector<edge*> eVec;
	cur.x = dst.x; cur.y = dst.y; cur._x = dst._x; cur._y = dst._y ;
	segment *seg;
	int index = 0;
	bool x_dir = false;				// checks if this path is in the x direction
	bool y_dir = false;				// checks if this path is in the y direction
	int i = rp.size()-1;
	while (cur.x != src.x || cur.y != src.y) {
		pair<int,int> next;
		//next.first = rp.at(i-1).x;
		//next.second = rp.at(i-1).y;
		next.first = cur._x; 
		next.second = cur._y;
		double weight = 0.0;
		int edgeIndex = 0;
		if (next.second == cur.y && next.first != cur.x) {	// move in along x-direction
					
			if (!x_dir) {		// not already moving in x-direction
				edgeIndex = 0;
				seg = new segment;
				seg->numEdges = 0;
				seg->weight = 0;
				seg->p1 = cur;	// set starting point of segment
				x_dir = true;
				y_dir = false;
				path.push_back(seg);
				if (index-1 >= 0) {
					path.at(index-1)->p2 = cur;
					int eVecSize = eVec.size();
					path.at(index-1)->edges = new edge[eVecSize];
					for (int k = 0; k < eVec.size(); k++) {
						path.at(index-1)->edges[k] = *eVec.at(k);
					}
					eVec.clear();
				}
				index++;
			}
			string key = static_cast<ostringstream*>(
				&(ostringstream() << cur.x << cur.y << next.first << next.second) )->str();
			edges[key]->util++;
			seg->numEdges++;
			if (edges[key]->cap == 0) { // this is a blocked edge with cap = 0, so update total overflow
				weight = (double) edges[key]->cap*EDGE_BLOCK_MULTIPLIER;
				rst->tof += 1; 
			}
			else {
				weight = (double) edges[key]->util / edges[key]->cap;
				if (edges[key]->util > edges[key]->cap) { // this is a blocked edge that is was previously full
					rst->tof += 1;
				}
			}
			edges[key]->weight = weight;
			seg->weight += weight;
			nets.at(netNum).croutes->weight += weight;
			eVec.push_back(edges[key]);
			edgeIndex++;
		}
		else if (next.second != cur.y && next.first == cur.x) {	// move along y-direction

			if (!y_dir) {		// not already moving in y-direction
				edgeIndex = 0;
				seg = new segment;
				seg->numEdges = 0;
				seg->weight = 0;
				seg->p1 = cur;	// set starting point of segment
				x_dir = false;
				y_dir = true;
				path.push_back(seg);
				if (index-1 >= 0) {
					path.at(index-1)->p2 = cur;
					int eVecSize = eVec.size();
					path.at(index-1)->edges = new edge[eVecSize];
					for (int k = 0; k < eVec.size(); k++) {
						path.at(index-1)->edges[k] = *eVec.at(k);
					}
					eVec.clear();
				}
				index++;
			}
			string key = static_cast<ostringstream*>(
				&(ostringstream() << cur.x << cur.y << next.first << next.second) )->str();
			edges[key]->util++;
			seg->numEdges++;
			if (edges[key]->cap == 0) { // this is a blocked edge with cap = 0, so update total overflow
				weight = (double) edges[key]->cap*EDGE_BLOCK_MULTIPLIER;
				rst->tof += 1; 
			}
			else {
				weight = (double) edges[key]->util / edges[key]->cap;
				if (edges[key]->util > edges[key]->cap) { // this is a blocked edge that is was previously full
					rst->tof += 1;
				}
			}
			edges[key]->weight = weight;
			seg->weight += weight;
			nets.at(netNum).croutes->weight += weight;
			eVec.push_back(edges[key]);
			edgeIndex++;
		}
		//point tmp ;
		//tmp.x = next.first;
		//tmp.y = next.second;
		//cur = tmp;
		cur = rp.at(i-1);
		i-=1;
	}
	path.at(index-1)->p2 = cur;

	// route
	nets.at(netNum).croutes->numSegs += path.size();

	return path;
}

/**
* A* Search Path
*/
vector<segment*> findPathUsingAStar(point s, point t, routingInst *rst, int netNum) 
{
	vector< pair<int,int> > processed;	
	rp.clear();
	pq.clear();
	// Initialize variables
	///point vParent;						// parent of current working vertex
	double dv = INT_MAX;				// weighted distance from s to v
	double fv = INT_MAX;			// score = dv + manhattan_distance(v, t)
	double du = 0.0;				// initially set to 0 distance between start to currPoint

	enqueue( make_pair(s, getDist(s,t)) );	// first enqueue start in the openset
	while( !pq.empty() ) {
		pair<point, double> currPoint = extractMin();
		rp.push_back(currPoint.first);
		du = (double) currPoint.second - getDist(currPoint.first,t);
		if (currPoint.first.x == t.x && currPoint.first.y == t.y) { // if reach destination
			return retrace(currPoint.first, s, rst, netNum);
		}
		dequeue();
		processed.push_back(make_pair(currPoint.first.x,currPoint.first.y));
		vector<point> curr_adj = findAdjacentVertices(currPoint.first, rst->gx, rst->gy);
		vector<point>::iterator it_adj;
		for (it_adj = curr_adj.begin(); it_adj != curr_adj.end(); it_adj++) {
			point v = (*it_adj);
			string key = static_cast<ostringstream*>(
				&(ostringstream() << currPoint.first.x << currPoint.first.y << v.x << v.y) )->str();
			
			double weight = 0.0;
			if (edges[key]->cap == 0) {	// this is a blocked edge with cap = 0
				weight = (double) EDGE_BLOCK_MULTIPLIER*(edges[key]->util+1);							
			}
			else {
				weight = (double) edges[key]->util + 1 / edges[key]->cap;		
			}
			double temp = du + weight ; 
			vector<pair<int,int>>::iterator it_proc;
			it_proc = find(processed.begin(), processed.end(), make_pair(v.x,v.y));
			if (it_proc != processed.end() && dv < temp) { // if this neightbor is already processed

			}

			else if (find_if(pq.begin(), pq.end(), FindFirst(v)) == pq.end() || dv > temp) { // if current's neighbor not already inqueue
				dv = temp;
				fv = dv + getDist(v, t);
				v._x = currPoint.first.x; v._y = currPoint.first.y;
				vector<pair<point, double>>::iterator iter;
				iter = find_if(pq.begin(), pq.end(), FindFirst(v));
				if (iter == pq.end()) {	// if it's not in the queue already
					enqueue(make_pair(v, fv));
				} 
				else {
					// remove the pair, update the score, and reinsert
					pair<point, double> newPair = make_pair((*iter).first, fv);
					pq.erase(iter);
					enqueue(newPair);
				}
			}

		}
	}
}


/**
* Compare function used in sort
*/
int compare(const net &a, const net &b) 
{
	int x = a.croutes->weight;
	int y = b.croutes->weight;
	return x > y;
}

/**
* Ripup and reroute process
*/
int reroute(routingInst *rst) 
{
	/*
	* ORDER ALL NETS n=1 to numNets based on decreasing values of weights
	*/
	cout << "Sorting nets by weights" << endl;
	sort(nets.begin(), nets.end(), compare);
	
	// Route all nets in order of the initial routing, assume n has two terminals
	for (int n = 0; n < rst->numNets; n++) {
		cout << "net " << nets.at(n).id << ": " << nets.at(n).croutes->weight << endl;

		// Rip up net n
		for (int i = 0; i < nets.at(n).croutes->numSegs; i++) {
			
			for (int j = 0; j < nets.at(n).croutes->segments->numEdges; j++) {

				edge e = nets.at(n).croutes->segments[i].edges[j];
				string key = static_cast<ostringstream*>(
						&(ostringstream() << e.p1.x << e.p1.y << e.p2.x << e.p2.y) )->str();
				edges[key]->util--;
				e.util--;
				if (e.cap == 0) { // this is a blocked edge of cap = 0
					rst->tof--;
					e.weight -= EDGE_BLOCK_MULTIPLIER;
					edges[key]->weight -= EDGE_BLOCK_MULTIPLIER;
				}
				else {
					if (e.util >= e.cap) { // this is still an edge that is over its capacity
						rst->tof--;
					}
					e.weight -= (double) 1 / rst->cap;
					edges[key]->weight -= (double) 1 / rst->cap;
				}
				
			}
			nets.at(n).croutes->segments[i].weight = 0;
			rst->nets[n].croutes->segments[i].weight = 0;	
		}

		//rst->nets[n].croutes->weight = 0;
		//nets.at(n).croutes->weight = 0;

		// Perform A* search path for net n
		// set up route var
		//route *r = new route;
		//vector<segment*> seg;
		//seg.clear();
		nets.at(n).croutes = new route;
		nets.at(n).croutes->numSegs = 0;
		nets.at(n).croutes->weight = 0;
		vector<segment*> seg;
		for (int i = 0; i < nets.at(n).numPinPts; i+=2) {
			vector<segment*> tmp = findPathUsingAStar(nets.at(n).pinPts[i], nets.at(n).pinPts[i+1], rst, n) ; 
			seg.insert(seg.end(), tmp.begin(), tmp.end());
		}
		int totalSeg = seg.size();
		nets.at(n).croutes->segments = new segment[totalSeg];
		for (int i = 0; i < totalSeg; i++) {
			nets.at(n).croutes->segments[i] = *seg.at(i);
		}
	}
	
	cout << "Total overflow is " << rst->tof << endl;
	return 1;
}

int readBenchmark(const char *fileName, routingInst *rst)
{
	//Blockage updates
	int blockages;

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
			e->weight = 0.0;
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
			e->cap = rst->cap;
			e->util = 0;
			e->weight = 0.0;
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
		nets.push_back(rst->nets[i]);			// add to net vector
	}

	
	cout << "DONE with net mapping" << endl;

	//Get Blockage Data
	getline(myfile, line);
	blockages = atoi(line.c_str());
	vector<string> buffer1, buffer2;
	int x1, y1, x2, y2, bCap;
	for (int i = 0; i < blockages; i++) {
		getline(myfile, line);
		buffer = split(line, '\t');
		buffer1 = split(buffer.at(0), ' ');
		buffer2 = split(buffer.at(1), ' ');
		x1 = atoi(buffer1.at(0).c_str());
		y1 = atoi(buffer1.at(1).c_str());
		x2 = atoi(buffer2.at(0).c_str());
		y2 = atoi(buffer2.at(1).c_str());
		bCap = atoi((buffer2.at(2).c_str()));

		string key = static_cast<ostringstream*>(
				&(ostringstream() << x1 << y1 << x2 << y2) )->str();
		edges[key]->cap = bCap;
	}
	cout << "Routing instance created with no errors!\n\n" << 
		"Grid summary:\nGrid dimension is " << rst->gx << " by " << rst->gy 
		<< "\nCapacity for normal edge is " << rst->cap 
		<< "\nTotal number of nets is " << rst->numNets
		<< "\nTotal number of blockages is " << blockages << endl;
	
	rst->tof = 0;
	rst->twl = 0;

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
	cout << "\nPerform initial routing. . . Please wait this can take a few mins" << endl;
	vector<segment> segments;
	vector<point> pinPts;

	for (int i = 0; i < rst->numNets; i++) {

		// Initialize stuff
		rst->nets[i].numCRoutes = 1; // for now, number of candidate route is just one
		route *r = new route;
		r->numSegs = 0;
		r->weight = 0;
		segments.clear();
		pinPts.clear();

		for (int j = 0; j < rst->nets[i].numPins; j++) {
			
			// if not connected
				
			if (!rst->nets[i].pins[j].isConnected) {
				rst->nets[i].pins[j].isConnected = true;
				pin thisPin = rst->nets[i].pins[j];
				pinPts.push_back(thisPin.loc);
				pin nextPin ;

				// find closest connected point to route to
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
				pinPts.push_back(nextPin.loc);

				// Route horizontally				
				int distX = getXDist(thisPin.loc, nextPin.loc);
				segment *seg = new segment;
				seg->p1 = thisPin.loc;				//define starting point
				point p;
				p.x = nextPin.loc.x;
				p.y = thisPin.loc.y;
				seg->p2 = p;
				int index = 0;
				double weight = 0.0;
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
					seg->numEdges++;	
					if (edges[key]->cap == 0) {	// this is a blocked edge with cap = 0, so update total overflow
						weight = (double) edges[key]->util*EDGE_BLOCK_MULTIPLIER;
						rst->tof += 1;								
					}
					else {
						weight = (double) edges[key]->util / edges[key]->cap;
						if (edges[key]->util > edges[key]->cap) {	// this is a blocked edge that is was previously full 
							rst->tof += 1;
						}
					}
					edges[key]->weight = weight;
					seg->edges[index] = *edges[key];
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
				index = 0;
				weight = 0.0;
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
						seg->numEdges++;
						if (edges[key]->cap == 0) { // this is a blocked edge with cap = 0, so update total overflow
							weight = (double) edges[key]->util*EDGE_BLOCK_MULTIPLIER;
							rst->tof += 1; 
						}
						else {
							weight = (double) edges[key]->util / edges[key]->cap;
							if (edges[key]->util > edges[key]->cap) { // this is a blocked edge that is was previously full
								rst->tof += 1;
							}
						}
						edges[key]->weight = weight;
						seg->edges[index] = *edges[key];
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
		for (int n = 0; n < totalSeg; n++) {
			r->segments[n] = segments.at(n);
		}

		rst->nets[i].croutes = r;	// for now croutes contains only one route
		nets.at(i).croutes = r;

		nets.at(i).numPinPts = pinPts.size();
		nets.at(i).pinPts = new point[nets.at(i).numPinPts];
		for (int n = 0; n < pinPts.size(); n++) {
			nets.at(i).pinPts[n] = pinPts.at(n);
		}
	}

	cout << "Done with initial routing" << endl;
	cout << "Initial total overflow is " << rst->tof << endl;
	return 1;
}

/**
* Solve routing 
**/
int solveRouting(routingInst *rst)
{
	int status = initialRoute(rst);	// Initial Routing to get net ordering based on weights of routes
	int iter;		// How many iterations to perform
	cout << endl;
	for (iter = 1; iter > 0; iter--) {
		cout << "Perform reroute iteration " << iter << endl;
		reroute(rst);
	}
	
	return 1;
}



int writeOutput(const char *outRouteFile, routingInst *rst)
{
  cout << "\nWriting Output to file " << "outRouteFile" << endl;
  ofstream out;
  out.open (outRouteFile);
	for (int i = 0; i < rst->numNets; i++) {
		out << "n" << nets.at(i).id << endl;
		route *croute = nets.at(i).croutes;
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
  


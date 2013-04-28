// ECE556 - Copyright 2013 University of Wisconsin-Madison.  All Rights Reserved.

#include "ece556.h"
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <stdlib.h>

int readBenchmark(const char *fileName, routingInst *rst){

	//Constraings of the Grid Graph	
	int GridDi_x, GridDi_y;
	int capacity;
	int num_nets;

	//Net Data
	char * net_name;
	int net_pins;
	int net_x, net_y;
	int i, j, k;

	//Blockage updates
	int blockages;
	int point1_x, point1_y;
	int point2_x, point2_y;
	int new_capacity;

	ifstream myfile;
	myfile.open(fileName);
	string line;
	string buffer;
	if (!myfile.good()){
		cout<<"Error: File could not open";
		return 0;
	}

	//Parse Grid Dimensions
	getline(myfile,line);
	buffer = strtok(line," ");
	GridDi_x = atoi(strtok(line," "));
	GridDi_y = atoi(strtok(line," "));
	//Get Capacity
	getline(myfile,line);
	buffer = strtok(line," ");
	capacity = atoi(strtok(line," "));
	//Get the Num Nets
	getline(myfile,line);
	buffer = (strtok(line," "));
	buffer = (strtok(line," "));
	num_nets = atoi(strtok(line," "));
	//Get Nets & Net Data
	for (i = 0; i < num_nets; i++){
		getline(myfile,line);
		net_name = strtok(line," ");
		net_pins = atoi(strtok(line," "));
		for (j = 0; j < net_pins; j++){
			getline(myfile,line);
			net_x = atoi(strtok(line," "));
			net_y = atoi(strtok(line," "));
		}
	}
	//Get Blockage Data
	getline(myfile,line);
	blockages = atoi(line);
	for (k = 0; k < blockages; k++){
		getline(myfile,line);
		point1_x = atoi(strtok(line," "));
		point1_y = atoi(strtok(line," "));
		point2_x = atoi(strtok(line," "));
		point2_y = atoi(strtok(line," "));
		new_capacity = atoi(strtok(line," "));
	}


  return 1;
}

int solveRouting(routingInst *rst){
  /*********** TO BE FILLED BY YOU **********/

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
  


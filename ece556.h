// ECE556 - Copyright 2013 University of Wisconsin-Madison.  All Rights Reserved.


#ifndef ECE556_H
#define ECE556_H

#include <stdio.h>
//#include <conio.h>
#include <iostream>
#include <map>

 /**
  * A structure to represent a 3D Point. 
  */
 typedef struct
 {
   int x ; /* x coordinate ( >=0 in the routing grid)*/
   int y ; /* y coordinate ( >=0 in the routing grid)*/
   
   int _x ; /* parent's coordinate */
   int _y ; /* parent's coordinate */

 } point ;


  typedef struct
 {
   point loc; /* x coordinate ( >=0 in the routing grid)*/
   bool isConnected ; /* y coordinate ( >=0 in the routing grid)*/

 } pin ;

   /**
  * A structure to represent an edge
  */
 typedef struct
 {
   point p1 ; 	
   point p2 ; 	
   
   int cap; 
   int util;
   double weight;

 } edge ;
 
  /**
  * A structure to represent a segment
  */
 typedef struct
 {
   point p1 ; 	/* start point of a segment */
   point p2 ; 	/* end point of a segment */
   
   int numEdges ; 	/* number of edges in the segment*/
   edge *edges ;  	/* array of edges' keys representing the segment*/
   double weight ;		
   
 } segment ;
 
 
  /**
  * A structure to represent a route
  */
  typedef struct
  {
    int numSegs ;  	/* number of segments in a route*/
    segment *segments ;  /* an array of segments (note, a segment may be flat, L-shaped or any other shape, based on your preference */
	double weight ;  

  } route ;
 
 
  /**
  * A structure to represent nets
  */
  typedef struct
  {

   int id ; 		/* ID of the net */
   int numPins ; 	/* number of pins (or terminals) of the net */
   pin *pins ; 	/* array of pins (or terminals) of the net. */

   int numCRoutes ; 	/* number of (candidate) routes of the net. This may be equal to one (only one candidate route) in your implementation. */
   route *croutes ;		/* array of candidate routes of the net. */

   int numPinPts;		/* number of sorted pins after initial routing phase to be used in RRR */
   point *pinPts;		/* array of sorted pins after initial routing phase to be used in RRR */

  } net ;
  
  /**
  * A structure to represent the routing instance
  */
  typedef struct
  {
   int gx ;		/* x dimension of the global routing grid */
   int gy ;		/* y dimension of the global routing grid */
   
   int cap ;
   
   int numNets ;	/* number of nets */
   net *nets;		/* array of nets */
   
   int numEdges ; 	/* number of edges of the grid */
   int *edgeCaps; 	/* array of the actual edge capacities after considering for blockages */
   int *edgeUtils;	/* array of edge utilizations */  
   
   int tof ;		/* total overflow */
   int twl;			/* total wirelength (not used atm) */
   
  } routingInst ;
  


/* int readBenchmark(const char *fileName, routingInst *rst)
   Read in the benchmark file and initialize the routing instance.
   This function needs to populate all fields of the routingInst structure.
   input1: fileName: Name of the benchmark input file
   input2: pointer to the routing instance
   output: 1 if successful
*/
int readBenchmark(const char *fileName, routingInst *rst);
  
/* int solveRouting(routingInst *rst)
   This function creates a routing solution
   input: pointer to the routing instance
   output: 1 if successful, 0 otherswise (e.g. the data structures are not populated) 
*/
int solveRouting(routingInst *rst);
  
/* int writeOutput(const char *outRouteFile, routingInst *rst)
   Write the routing solution obtained from solveRouting(). 
   Refer to the project link for the required output format.

   Finally, make sure your generated output file passes the evaluation script to make sure
   it is in the correct format and the nets have been correctly routed. The script also reports
   the total wirelength and overflow of your routing solution.

   input1: name of the output file
   input2: pointer to the routing instance
   output: 1 if successful, 0 otherswise(e.g. the output file is not valid) 
  */
  int writeOutput(const char *outRouteFile, routingInst *rst);
  
  /* int release(routingInst *rst)
     Release the memory for all the allocated data structures. 
     Failure to release may cause memory problems after multiple runs of your program. 
     Need to recursively delete all memory allocations from bottom to top 
     (starting from segments then routes then individual fields within a net struct, 
     then nets, then the fields in a routing instance, and finally the routing instance)

     output: 1 if successful, 0 otherswise 
  */
 int release(routingInst *rst);


#endif // ECE556_H


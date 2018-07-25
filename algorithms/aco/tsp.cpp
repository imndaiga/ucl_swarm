/***************************************************************************

    File's name: tsp.cpp

    TSP class contains the attributes and methods needed to read the TSPLIB
    files and compute the distances between the cities.
    There are four ways to compute the distances depending on the data in the
    instances (EDGE_WEIGHT_TYPE), i.e. EUC 2D, GEO, CEIL 2D, ATT.

	Methods implemented in TSP class:
		// Constructor
		TSP::TSP (const char *tsp_file_name){

		// Destructor
		TSP::~TSP (){

		// Compute distances using the coordinates in the instance file
		long int round_distance (long int i, long int j);
		long int ceil_distance (long int i, long int j);
		long int geo_distance (long int i, long int j);
		long int att_distance (long int i, long int j);

		// Computes the matrix of all intercity distances
		long int ** TSP::compute_distances(void)

		// Computes the distance between two cities
		long int TSP::compute_distance(long int i, long int j)

		// Get distance between cities i and j
		long int TSP::getDistance(long int i, long int j)

		// Print the distance matrix
		void TSP::printDistance(void)

		// Return the number of cities
		long int TSP::getSize()

***************************************************************************/


#include <math.h>
#include <limits.h>
#include "tsp.h"

using namespace std;


long int TSP::round_distance (long int i, long int j) 
/*    
      FUNCTION: compute Euclidean distances between two nodes rounded to next 
                integer for TSPLIB instances
      INPUT:    two node indices
      OUTPUT:   distance between the two nodes
      COMMENTS: for the definition of how to compute this distance see TSPLIB
*/
{
    double xd = nodeptr[i].x - nodeptr[j].x;
    double yd = nodeptr[i].y - nodeptr[j].y;
    double r  = sqrt(xd*xd + yd*yd) + 0.5;

    return (long int) r;
}

long int TSP::ceil_distance (long int i, long int j) 
/*    
      FUNCTION: compute ceiling distance between two nodes rounded to next 
                integer for TSPLIB instances
      INPUT:    two node indices
      OUTPUT:   distance between the two nodes
      COMMENTS: for the definition of how to compute this distance see TSPLIB
*/
{
    double xd = nodeptr[i].x - nodeptr[j].x;
    double yd = nodeptr[i].y - nodeptr[j].y;
    double r  = sqrt(xd*xd + yd*yd) + 0.000000001;

    return (long int)r;
}

long int TSP::geo_distance (long int i, long int j) 
/*    
      FUNCTION: compute geometric distance between two nodes rounded to next 
                integer for TSPLIB instances
      INPUT:    two node indices
      OUTPUT:   distance between the two nodes
      COMMENTS: adapted from concorde code
                for the definition of how to compute this distance see TSPLIB
*/
{
    double deg, min;
    double lati, latj, longi, longj;
    double q1, q2, q3;
    long int dd;
    double x1 = nodeptr[i].x, x2 = nodeptr[j].x, 
	y1 = nodeptr[i].y, y2 = nodeptr[j].y;

    deg = dtrunc (x1);
    min = x1 - deg;
    lati = M_PI * (deg + 5.0 * min / 3.0) / 180.0;
    deg = dtrunc (x2);
    min = x2 - deg;
    latj = M_PI * (deg + 5.0 * min / 3.0) / 180.0;

    deg = dtrunc (y1);
    min = y1 - deg;
    longi = M_PI * (deg + 5.0 * min / 3.0) / 180.0;
    deg = dtrunc (y2);
    min = y2 - deg;
    longj = M_PI * (deg + 5.0 * min / 3.0) / 180.0;

    q1 = cos (longi - longj);
    q2 = cos (lati - latj);
    q3 = cos (lati + latj);
    dd = (int) (6378.388 * acos (0.5 * ((1.0 + q1) * q2 - (1.0 - q1) * q3)) + 1.0);
    return dd;

}

long int TSP::att_distance (long int i, long int j) 
/*    
      FUNCTION: compute ATT distance between two nodes rounded to next 
                integer for TSPLIB instances
      INPUT:    two node indices
      OUTPUT:   distance between the two nodes
      COMMENTS: for the definition of how to compute this distance see TSPLIB
*/
{
    double xd = nodeptr[i].x - nodeptr[j].x;
    double yd = nodeptr[i].y - nodeptr[j].y;
    double rij = sqrt ((xd * xd + yd * yd) / 10.0);
    double tij = dtrunc (rij);
    long int dij;

    if (tij < rij)
        dij = (int) tij + 1;
    else
        dij = (int) tij;
    return dij;
}


long int ** TSP::compute_distances(void)
/*    
      FUNCTION: computes the matrix of all intercity distances
      INPUT:    none
      OUTPUT:   pointer to distance matrix, has to be freed when program stops
*/
{
    long int     i, j;
    long int     **matrix;

    if((matrix = (long int **) malloc(sizeof(long int) * n * n +
			sizeof(long int *) * n	 )) == NULL){
	fprintf(stderr,"Out of memory, exit.");
	exit(1);
    }
    for ( i = 0 ; i < n ; i++ ) {
	matrix[i] = (long int *)(matrix + n) + i*n;
	for ( j = 0  ; j < n ; j++ ) {
	    matrix[i][j] = compute_distance(i, j);
	}
    }
    return matrix;
}

long int TSP::compute_distance(long int i, long int j){
    if ( strcmp("EUC_2D", edge_weight_type) == 0 ) {
         return(round_distance(i,j));
    }
    else if ( strcmp("CEIL_2D", edge_weight_type) == 0 ) {
         return(ceil_distance(i,j));
    }
    else if ( strcmp("GEO", edge_weight_type) == 0 ) {
	 return(geo_distance(i,j));
    }
    else if ( strcmp("ATT", edge_weight_type) == 0 ) {
	 return(att_distance(i,j));
    }
    return(-1);

}

void TSP::printDistance(void) {
  long int i,j;

  printf("Distance Matrix:\n");
  for ( i = 0 ; i < n ; i++) {
    printf("From %ld:  ",i);
    for ( j = 0 ; j < n - 1 ; j++ ) {
      printf(" %ld", distance[i][j]);
    }
    printf(" %ld\n", distance[i][n-1]);
    printf("\n");
  }
  printf("\n");
}

double TSP::dtrunc (double x){
    int k;
    k = (int) x;
    x = (double) k;
    return x;
}

/* Return the number of cities*/
long int TSP::getSize(){
   return(n);
}

/* Get distance between cities i and j*/
long int TSP::getDistance(long int i, long int j){
   if(i>=n || i<0 || j>=n || j<0 ){
      printf("ERROR accessing distance.\n");
      exit(1);
   }     
   return(distance[i][j]);  //distance is a matrix containing all distances between every two cities (See compute_distances)
}

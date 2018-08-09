#ifndef TSP_H
#define TSP_H

#define LINE_BUF_LEN     100
//#define M_PI 3.14159265358979323846264

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <vector>
#include <iostream>

using namespace std;

struct point {
    double x;
    double y;
};

class TSP {
    public:
        //constructor
        TSP(const char *tsp_file_name){
            char         buf[LINE_BUF_LEN];
            long int     i, j;

            // Open file
            tsp_file = fopen(tsp_file_name, "r");
            if ( tsp_file == NULL ) {
                fprintf(stderr,"No instance file specified, abort\n");
                exit(1);
            }
            assert(tsp_file != NULL);
            printf("\nReading tsp-file %s ... \n", tsp_file_name);

            // Read lines
            fscanf(tsp_file,"%s", buf);
            while ( strcmp("NODE_COORD_SECTION", buf) != 0 ) {
                if ( strcmp("NAME", buf) == 0 ) {
                    fscanf(tsp_file, "%s", buf);
                    printf("%s ", buf);
                    fscanf(tsp_file, "%s", buf);
                    strcpy(name, buf);
                    printf("%s \n", name);
                    buf[0]=0;
                }
                else if ( strcmp("NAME:", buf) == 0 ) {
                    fscanf(tsp_file, "%s", buf);
                    strcpy(name, buf);
                    printf("%s \n", name);
                    buf[0]=0;
                }
                else if ( strcmp("COMMENT", buf) == 0 ){
                    fgets(buf, LINE_BUF_LEN, tsp_file);
                    printf("%s", buf);
                    buf[0]=0;
                }
                else if ( strcmp("COMMENT:", buf) == 0 ){
                    fgets(buf, LINE_BUF_LEN, tsp_file);
                    printf("%s", buf);
                    buf[0]=0;
                }
                else if ( strcmp("TYPE", buf) == 0 ) {
                    fscanf(tsp_file, "%s", buf);
                    printf("%s ", buf);
                    fscanf(tsp_file, "%s", buf);
                    printf("%s\n", buf);
                    if( strcmp("TSP", buf) != 0 ) {
                    fprintf(stderr,"\n Not a TSP instance in TSPLIB format !!\n");
                    exit(1);
                    }
                    buf[0]=0;
                }
                else if ( strcmp("TYPE:", buf) == 0 ) {
                    fscanf(tsp_file, "%s", buf);
                    printf("%s\n", buf);
                    if( strcmp("TSP", buf) != 0 ) {
                    fprintf(stderr,"\n Not a TSP instance in TSPLIB format !!\n");
                    exit(1);
                    }
                    buf[0]=0;
                }
                else if( strcmp("DIMENSION", buf) == 0 ){
                    fscanf(tsp_file, "%s", buf);
                    printf("%s ", buf);
                    fscanf(tsp_file, "%ld", &n);
                    printf("%ld\n", n);
                    assert ( n > 2 && n < 6000);
                    buf[0]=0;
                }
                else if ( strcmp("DIMENSION:", buf) == 0 ) {
                    fscanf(tsp_file, "%ld", &n);
                    printf("%ld\n", n);
                    assert ( n > 2 && n < 6000);
                    buf[0]=0;
                }
                else if( strcmp("DISPLAY_DATA_TYPE", buf) == 0 ){
                    fgets(buf, LINE_BUF_LEN, tsp_file);
                    printf("%s", buf);
                    buf[0]=0;
                }
                else if ( strcmp("DISPLAY_DATA_TYPE:", buf) == 0 ) {
                    fgets(buf, LINE_BUF_LEN, tsp_file);
                    printf("%s", buf);
                    buf[0]=0;
                }
                else if( strcmp("EDGE_WEIGHT_TYPE", buf) == 0 ){
                    buf[0]=0;
                    fscanf(tsp_file, "%s", buf);
                    printf("%s ", buf);
                    buf[0]=0;
                    fscanf(tsp_file, "%s", buf);
                    printf("%s\n", buf);
                    if ( strcmp("EUC_2D", buf) != 0 && strcmp("CEIL_2D", buf) != 0 && strcmp("GEO", buf) != 0 && strcmp("ATT", buf) != 0) {
                        fprintf(stderr,"EDGE_WEIGHT_TYPE %s not implemented\n",buf);
                    }
                    strcpy(edge_weight_type, buf);
                    buf[0]=0;
                }
                else if( strcmp("EDGE_WEIGHT_TYPE:", buf) == 0 ){
                    /* set pointer to appropriate distance function; has to be one of 
                    EUC_2D, CEIL_2D, GEO, or ATT. Everything else fails */
                    buf[0]=0;
                    fscanf(tsp_file, "%s", buf);
                    printf("%s\n", buf);
                    printf("%s\n", buf);
                    printf("%s\n", buf);
                    if ( strcmp("EUC_2D", buf) != 0 && strcmp("CEIL_2D", buf) != 0 && strcmp("GEO", buf) != 0 && strcmp("ATT", buf) != 0) {
                    fprintf(stderr,"EDGE_WEIGHT_TYPE %s not implemented\n",buf);
                    exit(1);
                    }
                    strcpy(edge_weight_type, buf);
                    buf[0]=0;
                }
                buf[0]=0;
                fscanf(tsp_file,"%s", buf);
            }


            if( strcmp("NODE_COORD_SECTION", buf) == 0 ){
                printf("found section contaning the node coordinates\n");
            } else {
                fprintf(stderr,"\n\nSome error ocurred finding start of coordinates from tsp file !!\n");
                exit(1);
            }

            if( (nodeptr = (point *) malloc(sizeof(struct point) * n)) == NULL ){
                exit(EXIT_FAILURE);
            } else {
                for ( i = 0 ; i < n ; i++ ) {
                    fscanf(tsp_file,"%ld %lf %lf", &j, &nodeptr[i].x, &nodeptr[i].y );
                }
            }
            printf(" number of cities is %ld\n",n);

            // Compute distances
            distance = compute_distances();

            printf("... done\n\n"); 
        };

        TSP(vector<vector<double>>& tsp_vector, string& tsp_units){
            long int     i, j;
            n = tsp_vector.size();
            double unit_mult;

            if(tsp_units == "m"){
                unit_mult = 1.;
            } else if(tsp_units == "cm") {
                unit_mult = 100.;
            }

            if( (nodeptr = (point *) malloc(sizeof(struct point) * n)) == NULL ){
                exit(EXIT_FAILURE);
            } else {
                for ( int i = 0 ; i < n ; i++ ) {
                    nodeptr[i].x = tsp_vector[i][0] * unit_mult;
                    nodeptr[i].y = tsp_vector[i][1] * unit_mult;
                }
            }

            // Compute distances
            distance = compute_distances();
        };

        //destructor
        ~TSP(){
            // printf("TSP destructor.");

            free( distance );
            distance =NULL;
            free( nodeptr );
            nodeptr=NULL;
        };

        char          name[LINE_BUF_LEN];      	   /* instance name */
        char          edge_weight_type[LINE_BUF_LEN];  /* selfexplanatory */
        long int      optimum;                /* optimal tour length if known, otherwise a bound */
        long int      n;                      /* number of cities */
        long int      n_near;                 /* number of nearest neighbors */
        struct point  *nodeptr;               /* array of structs containing coordinates of nodes */
        long int      **distance;	           /* distance matrix: distance[i][j] gives distance between city i und j */
        /*long int      **nn_list;              /* nearest neighbor list; contains for each node i a
                                                sorted list of n_near nearest neighbors */

        /* Distance functions*/
        long int     round_distance (long int i, long int j);
        long int     ceil_distance (long int i, long int j);
        long int     geo_distance (long int i, long int j);
        long int     att_distance (long int i, long int j);
        long int **  compute_distances(void);
        long int     compute_distance(long int i, long int j);

        static double dtrunc (double x);

        FILE *tsp_file;
        void printDistance(void) ;
        long int getSize();
        long int getDistance(long int i, long int j);

};

#endif
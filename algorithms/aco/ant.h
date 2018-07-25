#ifndef ANT_H
#define ANT_H

#include <limits.h>
#include "tsp.h"

class Ant {
    public:
        /* Default constructor*/
        Ant(){
            init = false;
        }
        Ant(TSP* tsp_arg, double ** prob_info, long int * pseed) {
            seed = pseed;
            size = tsp_arg->getSize();
            tsp  = tsp_arg;
            probability = prob_info;
            selection_prob = new double[size];
            tour = new long int[size];
            visited = new bool[size];
            tour_length = LONG_MAX;
            init = true;
        };
        /* Copy constructor*/
        Ant(Ant const& other) {
            seed = other.seed;
            size = other.size;
            tsp  = other.tsp;
            probability = other.probability;
            selection_prob = new double[size];
            tour = new long int[size];
            visited = new bool[size];
            for(int i=0; i<size ; i++){
                tour[i] = other.tour[i];
                visited[i] = other.visited[i];
                selection_prob[i] = other.selection_prob[i];
            }
            tour_length = other.tour_length;
            init=true;
        };
        /* Destructor */
        ~Ant() {
            if(init) {
                delete[] tour;
                delete[] visited;
                delete[] selection_prob;
            }
            init=false;
        }; 
        Ant& operator= (const Ant& other){
            seed = other.seed;
            size = other.size;
            tsp  = other.tsp;
            probability = other.probability;
            selection_prob = new double[size];
            tour = new long int[size];
            visited = new bool[size];
            for(int i=0; i<size ; i++){
                tour[i] = other.tour[i];
                visited[i] = other.visited[i];
                selection_prob[i] = other.selection_prob[i];
            }
            tour_length = other.tour_length;
            init=true;
        };

        bool init;
        long * seed;
        long int * tour;		/* solution */
        bool * visited;			/* auxiliary array for construction to keep track of visited cities */
        long int tour_length;
    
        double * selection_prob;  /*auxiliary array for selection*/
        double ** probability;    /*pointer to the pheremone of the colony!*/
        TSP * tsp;
        long int size;

        void computeTourLength(); //to implement
        void clearTour();
        long int getNextCity(long int i);
        long int getBestCity(long int i);
        
        void search(); //to implement
        long int getTourLength();
        long int getCity(long int i);
        void printTour();
        void checkTour();
};
#endif
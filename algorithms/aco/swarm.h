#ifndef SWARM_ACO_H
#define SWARM_ACO_H

#include <vector>
#include <ctime>
#include "ant.h"
#include "tsp.h"

using namespace std;

struct tsp_sol {
    long int * tour;
    long int tour_length;
};

class Swarm {
    public:
        /* Default constructor */
        Swarm();

        Swarm(int n_ants, char * instance_file, long int seed){
            this->alpha=1.0;
            this->beta=1.0;
            this->rho=0.2;
            this->n_ants=n_ants;
            this->max_iterations=0;
            this->max_tours=10000;
            this->instance_file=instance_file;
            this->seed = (long int) time(NULL);
            this->tsp = new TSP (this->instance_file);
        };

        Swarm(int n_ants, vector<vector<double>> instance_vector, long int seed, string tsp_units){
            this->alpha=1.0;
            this->beta=1.0;
            this->rho=0.2;
            this->n_ants=n_ants;
            this->max_iterations=0;
            this->max_tours=10000;
            this->instance_vector = instance_vector;
            this->seed = (long int) time(NULL);
            this->tsp = new TSP (this->instance_vector, tsp_units);
        };

        char * instance_file=NULL;
        vector<vector<double>> instance_vector;
        TSP* tsp;

        /*Probabilistic rule related variables*/
        double  ** pheromone;   	/* pheromone matrix */
        double  ** heuristic;  		/* heuristic information matrix */
        double  ** probability;    	/* combined value of pheromone X heuristic information */
        double  initial_pheromone=1.0;

        long int max_iterations;   //Max iterations
        long int iterations=0;           
        long int max_tours;        //Max tours
        long int tours=0;
        double   alpha;
        double   beta;
        double   rho;
        long int n_ants;
        long int seed = -1;

        vector<Ant> colony;		// Colony is a vector of containing all ants, each ant is represented as a vector
        Ant best_ant;
        long int best_tour_length=LONG_MAX;     /* length of the shortest tour found */

        void initializePheromone(double initial_value);
        void initializeHeuristic(); 
        void initializeProbabilty(); 
        void calculateProbability(); 
        void createColony();
        void evaporatePheromone();
        void addPheromone(long int i , long int j, double delta);
        void depositPheromone();
        bool terminationCondition();
        void freeMemory();
        tsp_sol optimize();
};

#endif
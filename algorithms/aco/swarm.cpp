#include "swarm.h"
#include <iostream>
#include <math.h>

using namespace std;

void Swarm::createColony() {
    // cout << "Creating colony.\n\n";
    for (int i = 0 ; i < n_ants ; i++ ) {
       // Add element at the end
       this->colony.push_back(Ant(this->tsp, this->probability, &seed)); // See class Ant
    }
};

void Swarm::initializePheromone( double initial_value ) {
    long int size = tsp->getSize();
    this->pheromone = new double * [size];
    for (int i = 0 ; i < size ; i++ ) {
        this->pheromone[i] = new double [size];
        for (int j = 0  ; j < size ; j++ ) {
            if (i==j) this->pheromone[i][i] = 0.0;
            else this->pheromone[i][j] = initial_value;
            /* symmetric TSP instances; hence this->pheromone[i][j] = this->pheromone[j][i] */
        }
    }
};

void Swarm::initializeHeuristic() {
    long int size = this->tsp->getSize();

    this->heuristic = new double * [size];
    for (int i = 0 ; i < size ; i++ ) {
        this->heuristic[i] = new double [size];
        for (int j = 0  ; j < size ; j++ ) {
            if (i!=j) {
                this->heuristic[i][j] = 1.0 / (double) this->tsp->getDistance(i,j);
                /* symmetric TSP instances; hence this->phermone[i][j] = this->pheromone[j][i] */
            } else{
                this->heuristic[i][j] = 0.0;
            }
        }
    }
};

void Swarm::initializeProbabilty() {
    long int size = tsp->getSize();
  
    this->probability = new double * [size];
    for (int i = 0 ; i < size ; i++ ) {
        this->probability[i] = new double [size];
        for (int j = 0  ; j < size ; j++ ) {
        this->probability[i][j] = 0.0;
        }
    }
};

void Swarm::calculateProbability() {
    long int size = this->tsp->getSize();
    for (int i = 0 ; i < size ; i++ ) {
        this->probability[i][i] = 0.0;
        for (int j = (i+1)  ; j < size ; j++ ) {
        this->probability[i][j] = pow(this->pheromone[i][j], this->alpha) * pow(this->heuristic[i][j], this->beta);
        /* symmetric TSP instances; hence phermone[i][j] = pheromone[j][i] */
        this->probability[j][i] = this->probability[i][j];
        }
    }
};

void Swarm::evaporatePheromone() {
    long int size = this->tsp->getSize();
    for (int i =0 ; i < size ; i ++) {
        for (int j=i+1 ; j < size ; j++) {
            this->pheromone[i][j] = (double)(1.0-this->rho) * this->pheromone[i][j];
            this->pheromone[j][i] = this->pheromone[i][j];
        }
    }
};

void Swarm::addPheromone(long int i , long int j, double delta) {
    this->pheromone[i][j] = this->pheromone[i][j] + delta;
    this->pheromone[j][i] = this->pheromone[j][i] + delta;
};

void Swarm::depositPheromone(){
    long int size = this->tsp->getSize();
    double deltaf;

    for (int i=0; i< n_ants; i++) {
        deltaf = 1.0 / (double) this->colony[i].getTourLength();
        for (int j =1 ; j < size ; j ++) {
            addPheromone(this->colony[i].getCity(j-1), this->colony[i].getCity(j), deltaf);
        }
        addPheromone(this->colony[i].getCity(size-1), this->colony[i].getCity(0), deltaf);
    }
};

bool Swarm::terminationCondition(){
    if (this->max_tours != 0 && this->tours >= this->max_tours){
        return(true);
    }
    if (this->max_iterations !=0 && this->iterations >= this->max_iterations){
        return(true);
    }
    return(false);
};

void Swarm::freeMemory(){
    for(int i=0; i < tsp->getSize(); i++) {
        delete[] pheromone[i];
        delete[] heuristic[i];
        delete[] probability[i];
    }
    delete tsp;
    delete[] pheromone;
    delete[] heuristic;
    delete[] probability;
}

struct tsp_sol Swarm::optimize(){

    initializePheromone(this->initial_pheromone);
    initializeHeuristic(); 
    initializeProbabilty(); 
    calculateProbability(); 
    createColony();

    //Iterations loop
    while(!terminationCondition()){
        //build solutions
        for(int i=0; i< this->n_ants; i++){
            //Construct solution
            this->colony[i].search();
            /*Check for new local optimum*/
            if(this->best_tour_length > this->colony[i].getTourLength()){
                this->best_tour_length = this->colony[i].getTourLength();
                this->best_ant = (Ant) this->colony[i];
            }
            this->tours++;
        }
        //Print convergence information
        // cout << "* " << this->tours << " : " << this->best_ant.getTourLength() << endl; 
        evaporatePheromone();//to implement
        depositPheromone();//to implement
        calculateProbability();//to implement
        this->iterations++;
    }
    //print out the best result found
    // cout << "Tour number: " << this->tours << endl;
    this->best_ant.printTour();

    freeMemory();   // Free memory.
    // cout << "\nEnd ACO execution.\n" << endl;

    tsp_sol sol;
    sol.tour = this->best_ant.tour;
    sol.tour_length = this->best_ant.tour_length;

    return(sol);
};
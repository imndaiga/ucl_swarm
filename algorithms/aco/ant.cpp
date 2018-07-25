/***************************************************************************

    File's name: ant.cpp

    Ant class provides ants with the routines and functions needed to construct
    solutions. Do not forget that in ACO, ants are general procedures to
    construct solutions stochastically, in which one transition rule is used
    to select one component to add to a partial solution.



***************************************************************************/

#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <string.h>

#include "utils.h"
#include "ant.h"

using namespace std;

/*Generate tour using probabilities*/
void Ant::search() {
  clearTour();
  
  tour[0] = (long int)(ran01(seed) * (double) size);
  visited[tour[0]] = true;
  // Select each city
  for (long int i = 1; i < size; i++) {
    tour[i] = getNextCity(tour[i-1]);
    visited[tour[i]] = true;
  }
  computeTourLength();
  printTour();
  checkTour();
}

/*Compute the length of tour*/
/* ! The tours start and end in the SAME city*/
void Ant::computeTourLength() {
  tour_length = 0;
  for (long int i=0; i <(size-1); i++) {
    tour_length = tour_length + tsp->getDistance(tour[i], tour[i+1]);
  }
  tour_length = tour_length + tsp->getDistance(tour[size-1], tour[0]);
}

long int Ant::getTourLength(){
  return(tour_length); //After a tour in completed this value is recomputed by every ant
}

/*Cleans the tour structures*/
void Ant::clearTour() {
  for (long int i = 0; i < size; i++) {
    visited[i]=false;
    tour[i]=-1;
    selection_prob[i]=0;
  }
}

/*Obtains the next city to visit after city i,
  according to the random proportional rule*/
long int Ant::getNextCity(long int i) {
  double sum_prob = 0.0;
  double choice;
  long int j;
  
  for (j=0; j < size; j++) {
    /*Calculate the probability of selecting each non-visited city*/
    if (!visited[j] && i!=j){
      sum_prob = sum_prob + probability[i][j];
      selection_prob[j] = sum_prob;
    } else {
      selection_prob[j] = 0.0;
    }
  }
  
  /*Choose a city*/
  if (sum_prob <= 0.0) {
    /*All zero prob, choose best available*/
    j = getBestCity(i);
  } else{
    choice = ran01(seed) * sum_prob;
    j = 0;
    while (choice > selection_prob[j]) j++;
  }
  return(j);
}

/*Obtains the next city to visit after city i,
 according to the random proportional rule*/
long int Ant::getBestCity(long int i) {
  long int j, selected=-1;
  long int best_distance=LONG_MAX;

  for (j=0; j< size; j++){ 
    if (!visited[j] && i!=j) {
      if (best_distance > tsp->getDistance(i, j)){ //Get the city with shortest distance
        best_distance = tsp->getDistance(i, j);
        selected =j;
      }
    }
  }
  return(selected);
}

/*Get the city in position i of the tour*/
long int Ant::getCity(long int i) {
  return(tour[i]);
}

void Ant::printTour() {
  for (long int i=0; i < size; i++) {
    printf("%ld - ", tour[i]);
  }
  printf("%ld Total cost: %ld \n", tour[0], tour_length);

}

/*Check if the tour is valid*/
void Ant::checkTour() {
  long int aux;
  for (long int i =0; i< size ; i++) {
    aux = tour[i];
    if (tour[i]>= size) cout << "Error: city " << i <<"has a bigger index than number of cities: " << aux << "endl";
    if (tour[i]< 0) cout << "Error: city " << i <<"has a negative index: " << aux << "endl";
    for (long int j=i+1; j<size; j++) {
      if (tour[i]==tour[j]) cout << "Error: city " << i <<"has same index than city "<< j << ": " << aux << "endl";
    }
  }
}

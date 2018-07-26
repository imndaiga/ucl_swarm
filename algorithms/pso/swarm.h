#ifndef SWARM
#define SWARM

#include "particle.h"
#include "node.h"
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <argos3/core/utility/math/vector3.h>

struct tsp_sol {
    long int * tour;
    long int tour_length;
};

class Swarm{
public:
  Swarm(int particle_count, float self_trust, float past_trust, float global_trust);

  void read_graph_definition(std::string filename);
  void load_test();
  void load_tsp(std::vector< argos::CVector2 > plantList, std::string units);
  tsp_sol optimize();

  std::vector<Particle> particles;
  std::vector<Node> nodes;
  double best_value;
  Position best_position;
private:
  bool normal_search();
  bool lazy_descent();
  bool energetic_descent();

  void particles_back_to_best();
  bool move_all_slowly();

  std::string trim(std::string);
  void assign_particle_positions();
  Position shuffle();

  double self_trust;
  double past_trust;
  double global_trust;
  int particle_count;
};

#endif

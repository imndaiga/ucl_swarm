#include "pso.h"

PSO::PSO(int particle_count, double self_trust, double past_trust, double global_trust){
    this->particle_count = particle_count;
    this->self_trust = self_trust;
    this->past_trust = past_trust;
    this->global_trust = global_trust;
};

void PSO::loadTSP(std::vector<argos::CVector2> tspList, std::string tspUnits){
    this->nodes.clear();

    for(int p=0; p < tspList.size(); p++) {
        struct PSO::Node n;

        if(tspUnits == "m") {
            n.x = tspList[p].GetX();
            n.y = tspList[p].GetY();
        } else if(tspUnits == "km") {
            n.x = tspList[p].GetX() * 0.001;
            n.y = tspList[p].GetY() * 0.001;
        } else if(tspUnits == "cm") {
            n.x = tspList[p].GetX() * 100.;
            n.y = tspList[p].GetY() * 100.;
        } else if(tspUnits == "mm") {
            n.x = tspList[p].GetX() * 1000.;
            n.y = tspList[p].GetY() * 1000.;
        }

        this->nodes.push_back(n);
    }
    initializeParticles();
}

void PSO::initializeParticles(){
    this->particles.clear();

    for(size_t i=0; i < this->particle_count; i++) {
        struct PSO::Particle p{};
        p.position = shuffle();
        this->particles.push_back(p);
        this->particles[i].position = shuffle();

        double curr_cost = this->particles[i].calculateCost();

        if(i==0 || this->best_value > curr_cost) {
            this->best_value = curr_cost;
            this->best_position = this->particles[i].position;
        }
    }
}

struct PSO::Position PSO::shuffle(){
    // Perform Knuth-Fisher-Yates shuffle
    Position p;
    std::vector<struct PSO::Node> new_vec(this->nodes);

    for(int i = new_vec.size() - 1; i > 0; i--){
        int n = rand() % (i + 1);
        PSO::Node tmp = new_vec[i];
        new_vec[i] = new_vec[n];
        new_vec[n] = tmp;
    }

    p.nodes = new_vec;
    return p;
}

double PSO::optimize(){
    double tmp;

    for(int i = 0; i < this->particles.size(); i++){
        this->particles[i].updateVelocity(this->best_position);

        tmp = this->particles[i].updatePosition();

        if(this->best_value > tmp){
        this->best_value = tmp;
        this->best_position = this->particles[i].position;
        }
        return this->best_value;
    }
}

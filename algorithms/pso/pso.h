#ifndef PSO_H
#define PSO_H

#include <argos3/core/utility/math/vector2.h>

class PSO {
    public:
        PSO(int particle_count, double self_trust, double past_trust, double global_trust);

        struct Node {
            double x;
            double y;
            int index;
            double distanceTo(Node alt) {
                int dist_x = this->x - alt.x;
                int dist_y = this->y - alt.y;
                double dist = std::sqrt((x*x) + (y*y));
                return dist;
            }
        };

        struct Velocity {
            int size;
            std::vector<int> from;
            std::vector<int> to;

            void addTransposition(int a, int b){
                this->size += 1;
                this->from.push_back(a);
                this->to.push_back(b);
            }

            struct Velocity operator*(const double& rhs) {
                double d = rhs;
                std::vector<int> from(this->from);
                std::vector<int> to(this->to);

                this->from.clear();
                this->to.clear();
                int size = this->size;
                this->size = 0;

                while( d >= 2 ){
                    for(int i = 0; i < size; i++){
                        this->addTransposition(from[i],to[i]);
                    }
                    d--;
                }

                return *this;
            }
        };

        struct Position {
            std::vector<Node> nodes;

            struct Position operator-(struct Position& rhs){
            }

            struct Position operator*(const double& rhs)
        };

        struct Particle {
            struct Velocity velocity;
            struct Position position;

            struct Position best_found;
            double best_value;
            struct Position best_position;

            double self_trust;
            double past_trust;
            double global_trust;

            double calculateCost() {
                double cost = 0.;
                double node_count = position.nodes.size();

                for(size_t n=0; n<node_count; n++) {
                    Node start_node = position.nodes.at(n);
                    Node end_node;

                    cost += start_node.distanceTo(end_node);
                }

                return cost;
            }

            double updatePosition() {
                // position += velocity;

                double new_cost = this->calculateCost();

                if(new_cost < this->best_value || this->best_value < 0){
                    this->best_value = new_cost;
                    this->best_position = position;
                }

                return this->best_value;
            }

            void updateVelocity(Position global_best_position) {
                Velocity a;

                if(this->velocity.size > 0){
                    a = (this->velocity * this->self_trust);

                    Velocity b = ((this->best_position - this->position) * this->past_trust);
                    // Velocity c = ((global_best - this->position) * this->global_trust);

                    // //std::cout << a.to_string();
                    // this->velocity =  Velocity(a + b + c);
                }
            }
        };

        void loadTSP(std::vector<argos::CVector2> tspList, std::string tspUnits);

        void initializeParticles();

        Position shuffle();

        double optimize();

        std::vector<Particle> particles;
        std::vector<Node> nodes;
    private:
        int particle_count;
        double self_trust;
        double past_trust;
        double global_trust;

        //These should add up to 1.0
        double best_value;
        Position best_position;
};

#endif
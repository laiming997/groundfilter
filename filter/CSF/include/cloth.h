#ifndef _CLOTH_H_
#define _CLOTH_H_

#include <math.h>
#include <vector>
#include <iostream>
#include <omp.h>
#include <sstream>
#include <list>
#include <cmath>
#include <string>
#include <queue>

#include "Vec3.h"
#include "Particle.h"

// post processing is only for connected component which is large than 50
#define MAX_PARTICLE_FOR_POSTPROCESSIN    50
struct XY {
    XY(int x1, int y1) {
        x = x1; y = y1;
    }

    int x;
    int y;
};

class cloth
{
private:
    // total number of particles is num_particles_width * num_particles_height
    int constraint_iterations;

    int rigidness;
    double time_step;

    std::vector<Particle> particles; // all particles that are part of this cloth

    double smoothThreshold;
    double heightThreshold;
    
public:
    Vec3 origin_pos;
    double step_x, step_y;
    std::vector<double> heightvals; // height values
    int num_particles_width;   // number of particles in width direction
    int num_particles_height;  // number of particles in height direction
    
    Particle* getParticle(int x, int y){
        return &particles[y * num_particles_width + x];
    }
    
    void makeConstraint(Particle *p1, Particle *p2) {
        p1->neighborsList.push_back(p2);
        p2->neighborsList.push_back(p1);
    }
    
public:
    int getSize(){
        return num_particles_width * num_particles_height;
    }
    
    std::size_t get1DIndex(int x, int y) {
        return y * num_particles_width + x;
    }
    
    inline std::vector<double>& getHeightvals() {
        return heightvals;
    }
    
    Particle* getParticle1d(int index) {
        return &particles[index];
    }
    
public:
    /* This is a important constructor for the entire system of
     * particles and constraints */
    cloth(const Vec3& origin_pos,
        int         num_particles_width,
        int         num_particles_height,
        double      step_x,
        double      step_y,
        double      smoothThreshold,
        double      heightThreshold,
        int         rigidness,
        double      time_step);
    ~cloth() {}
    
    /* this is an important methods where the time is progressed one
     * time step for the entire cloth.  This includes calling
     * satisfyConstraint() for every constraint, and calling
     * timeStep() for all particles
     */
    double timeStep();
    
    /* used to add gravity (or any other arbitrary vector) to all
     * particles */
    void addForce(const Vec3 direction);
    
    void terrCollision();
    void movableFilter();
    std::vector<int> findUnmovablePoint(std::vector<XY> connected);
    void  handle_slop_connected(std::vector<int>          edgePoints,
                                std::vector<XY>           connected,
                                std::vector<std::vector<int> > neibors);
    void saveToFile(std::string path = "");
    void saveMovableToFile(std::string path = "");
};


#endif
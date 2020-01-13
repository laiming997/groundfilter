#include "cloth.h"
#include <fstream>

cloth::cloth(const Vec3& _origin_pos,
            int         _num_particles_width,
            int         _num_particles_height,
            double      _step_x,
            double      _step_y,
            double      _smoothThreshold,
            double      _heightThreshold,
            int         rigidness,
            double      time_step):
    constraint_iterations(rigidness),
    time_step(time_step),
    smoothThreshold(_smoothThreshold),
    heightThreshold(_heightThreshold),
    origin_pos(_origin_pos),
    step_x(_step_x),
    step_y(_step_y),
    num_particles_width(_num_particles_width),
    num_particles_height(_num_particles_height){
    
    particles.resize(num_particles_width * num_particles_height);
    double time_step2 = time_step * time_step;
    // creating particles in a grid of particles from (0,0,0) to
    // (width,-height,0) creating particles in a grid
    for (int i = 0; i < num_particles_width; i++)
    {
        for (int j = 0; j < num_particles_height; j++)
        {
            Vec3 pos(origin_pos.f[0] + i * step_x,
                    origin_pos.f[1],
                    origin_pos.f[2] + j * step_y);
            
            // insert particle in column i at j'th row
            particles[j*num_particles_width + i] = Particle(pos, time_step2);
            particles[j*num_particles_width + i].pos_x = i;
            particles[j*num_particles_width + i].pos_y = j;
        }
    }
    // Connecting immediate neighbor particles with constraints
    // (distance 1 and sqrt(2) in the grid)
    for (int x = 0; x < num_particles_width; x++)
    {
        for (int y = 0; y < num_particles_height; y++)
        {
            if(x < num_particles_width - 1)   makeConstraint(getParticle(x,y), getParticle(x + 1, y));
            
            if(y < num_particles_height - 1)  makeConstraint(getParticle(x,y), getParticle(x, y + 1));
            
            if((x < num_particles_width - 1) && (y < num_particles_height - 1))  
                makeConstraint(getParticle(x,y), getParticle(x + 1, y + 1));
            
            if((x < num_particles_width - 1) && (y < num_particles_height - 1))  
                makeConstraint(getParticle(x + 1,y), getParticle(x, y + 1));
        }
    }
    
    // Connecting secondary neighbors with constraints (distance 2 and sqrt(4) in the grid)
    for (int x = 0; x < num_particles_width; x++)
    {
        for (int y = 0; y < num_particles_height; y++)
        {
            if (x < num_particles_width - 2)
                makeConstraint(getParticle(x, y), getParticle(x + 2, y));

            if (y < num_particles_height - 2)
                makeConstraint(getParticle(x, y), getParticle(x, y + 2));

            if ((x < num_particles_width - 2) && (y < num_particles_height - 2))
                makeConstraint(getParticle(x, y), getParticle(x + 2, y + 2));

            if ((x < num_particles_width - 2) && (y < num_particles_height - 2))
                makeConstraint(getParticle(x + 2, y), getParticle(x, y + 2));        
        }
    }
}
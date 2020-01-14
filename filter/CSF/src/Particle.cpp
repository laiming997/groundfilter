#include "Particle.h"

void Particle::timeStep()
{
    if(movable)
    {
        Vec3 temp = pos;
        pos = pos + (pos - old_pos) * (1.0 - DAMPING) + acceleration * time_step2;
        old_pos = temp;
    }
}

void Particle::satisfyConstraintSelf(int constraintTimes)
{
    Particle *p1 = this;
    for (int i = 0; i < neighborsList.size(); i++)
    {
        Particle *p2 = neighborsList[i];
        Vec3 correctionVector(0 , p2->pos.f[1] - p1->pos.f[1], 0);
        if(p1->isMovable() && p2->isMovable())
        {
            Vec3 correctionVectorHalf = correctionVector * (
                    constraintTimes > 14 ? 0.5 : doubleMove1[constraintTimes]);
            p1->offsetPos(correctionVectorHalf);
            p2->offsetPos(-correctionVectorHalf);
        }
        else if(p1->isMovable() && !p2->isMovable())
        {
            Vec3 correctionVectorHalf = correctionVector * (
                    constraintTimes > 14 ? 1 : singleMove1[constraintTimes]);
            p1->offsetPos(correctionVectorHalf);
        }
        else if(!p1->isMovable() && p2->isMovable())
        {
            Vec3 correctionVectorHalf = correctionVector * (
                    constraintTimes > 14 ? 1 : singleMove1[constraintTimes]);
            p2->offsetPos(-correctionVectorHalf);
        }
    }
}
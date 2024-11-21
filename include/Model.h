#ifndef _MODEL_H_
#define _MODEL_H_

#include <Ground.h>
#include <btBulletDynamicsCommon.h>

class Model :public Ground
{
protected:
public:
    void update_position();
    void  create(btScalar mass, btVector3 fallInertia);
};

#endif
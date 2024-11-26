#ifndef _MODEL_H_
#define _MODEL_H_

#include <Ground.h>
#include <btBulletDynamicsCommon.h>

class Model :public Ground
{

protected:
    btRigidBody* ground;        //µÿ√Ê
public:
    void update_position();
    void create(btScalar mass, btVector3 fallInertia,int shapetype, btVector3 dimensions);
    void key_callback(btDiscreteDynamicsWorld* dynamicsWorld, int key,int action, int mode);
    void moveModel(btDiscreteDynamicsWorld* dynamicsWorld,float dx, float dy, float fz);
    void rotateObject(float);
    void set_ground(btRigidBody* m_ground);

};

#endif
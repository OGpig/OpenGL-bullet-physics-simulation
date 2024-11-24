#include "Ground.h"
#include <btBulletDynamicsCommon.h>

void Ground::create()
{

    collisionShape = new btStaticPlaneShape(btVector3(0, 1, 0),translation.y);
    motionState = new btDefaultMotionState(btTransform(btQuaternion(0, rotation.y, rotation.z, 1), btVector3(translation.x, translation.y, translation.z)));
    //printf("x:%f   y:%f   z:%f\n", rotation.x, rotation.y, rotation.z);
    //printf("x:%f   y:%f   z:%f\n", translation.x, translation.y, translation.z);
    btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, motionState, collisionShape);
    rigidBody = new btRigidBody(groundRigidBodyCI);
}

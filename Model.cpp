#include"Model.h"
#include <btBulletDynamicsCommon.h>

void Model::create(btScalar mass, btVector3 fallInertia)
{
    collisionShape = new btSphereShape(1);
    motionState = new btDefaultMotionState(btTransform(btQuaternion(0, rotation.y, rotation.z, 1), btVector3(translation.x, translation.y, translation.z)));
    collisionShape->calculateLocalInertia(mass, fallInertia);
    btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(mass, motionState, collisionShape,fallInertia);
    rigidBody = new btRigidBody(groundRigidBodyCI);
}

void Model::update_position()
{
    btTransform trans;
    rigidBody->getMotionState()->getWorldTransform(trans);
    setTranslation(glm::vec3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()));
    //printf("x:%f   y:%f   z:%f\n", trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
    //printf("x:%f   y:%f   z:%f\n", scale.x, scale.y, scale.z);
}
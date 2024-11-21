#ifndef _GROUND_H_
#define _GROUND_H_
#include <btBulletDynamicsCommon.h>
#include"TriMesh.h"

class Ground :public TriMesh
{
public:
    btRigidBody* rigidBody;             //刚体
protected:
    btCollisionShape* collisionShape;   //碰撞形状
    btDefaultMotionState* motionState;  //运动状态
public:
    void create();
};
#endif

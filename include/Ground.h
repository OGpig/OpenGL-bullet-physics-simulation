#ifndef _GROUND_H_
#define _GROUND_H_
#include <btBulletDynamicsCommon.h>
#include"TriMesh.h"

class Ground :public TriMesh
{
public:
    btRigidBody* rigidBody;             //����
protected:
    btCollisionShape* collisionShape;   //��ײ��״
    btDefaultMotionState* motionState;  //�˶�״̬
public:
    void create();
};
#endif

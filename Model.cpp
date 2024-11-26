#include"Model.h"
#include <btBulletDynamicsCommon.h>

// �����Զ���Ļص���
class MyContactResultCallback : public btCollisionWorld::ContactResultCallback {
public:
    bool hasCollision = false;
    btRigidBody* tempRigidBody;  // ����Ŀ������ָ��
    btCollisionObject* originalBody;  // ����ԭʼ�����ָ��
    btCollisionObject* ground;
    MyContactResultCallback(btRigidBody* body) : tempRigidBody(body), originalBody(nullptr) {}

    // ����ԭʼ���壨�������ų����Լ���ײ��
    void setOriginalBody(btCollisionObject* body) {
        originalBody = body;
    }
    void set_ground(btCollisionObject* _ground)
    {
        ground = _ground;
    }

    btScalar addSingleResult(btManifoldPoint& point, const btCollisionObjectWrapper* colObj0, int partId0, int index0,
        const btCollisionObjectWrapper* colObj1, int partId1, int index1) override {
        // ȷ���ų����������ײ
        if (colObj0->getCollisionObject() != ground && colObj1->getCollisionObject() != ground) {
            if (colObj0->getCollisionObject() != originalBody && colObj1->getCollisionObject() != originalBody) {
                if (colObj0->getCollisionObject() != originalBody && colObj1->getCollisionObject() != originalBody) {
                    // �����ײ�����Ƿ�ΪĿ�����
                    if (colObj0->getCollisionObject() == tempRigidBody || colObj1->getCollisionObject() == tempRigidBody) {
                        if (point.getDistance() < 0) {
                            hasCollision = true;
                        }
                    }
                }
            }
        }
        return 0;
    }
};

void Model::set_ground(btRigidBody* m_ground)
{
    ground = m_ground;
}


void Model::create(btScalar mass, btVector3 fallInertia, int shapetype=0, btVector3 dimensions = btVector3(1, 1, 1))
{

    switch (shapetype) {
    case 0: // ����
        collisionShape = new btSphereShape(dimensions.x()); // ʹ�� x ��Ϊ�뾶
        break;
    case 1: // ����
        collisionShape = new btCapsuleShape(dimensions.x(), dimensions.y());
        // x �ǰ뾶, y �ǳ��ȣ����������˰���
        break;
    case 2: // Բ��
        collisionShape = new btCylinderShape(dimensions);
        // dimensions �� x/y/z �ֱ��ǰ뾶�͸߶ȵķ���
        break;
    case 3: // ����
        collisionShape = new btBoxShape(dimensions); // ���Ӱ�߳�
        break;
    }
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
    // ��ȡ��Ԫ��
    btQuaternion rotation = trans.getRotation();
    // ����Ԫ��ת��Ϊŷ���� (ZYX ˳��)
    btScalar yaw, pitch, roll; // Yaw (Z), Pitch (Y), Roll (X)
    rotation.getEulerZYX(yaw, pitch, roll);
    // ��ŷ���Ƿ�װ�� glm::vec3 �У���Ҫת��Ϊ������
    glm::vec3 eulerAngles(glm::degrees(roll), glm::degrees(pitch), glm::degrees(yaw));
    // ������ת
    setRotation(eulerAngles);
    //printf("x:%f   y:%f   z:%f\n", trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
    //printf("x:%f   y:%f   z:%f\n", scale.x, scale.y, scale.z);
}
//���̲���
void Model::key_callback(btDiscreteDynamicsWorld* dynamicsWorld, int key, int action, int mode)
{
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) {
        case GLFW_KEY_W: // ��ǰ
            moveModel(dynamicsWorld, 0, 0, -1);
            break;
        case GLFW_KEY_D:
            moveModel(dynamicsWorld, 0, 0, 1);
            break;
        // ��ת
        case GLFW_KEY_Q: rotateObject(10); break;  // ��ʱ��
        case GLFW_KEY_E: rotateObject(-10); break; // ˳ʱ��

        }
    }
}

//��ײ���
bool checkCollisionAt(btDiscreteDynamicsWorld* dynamicsWorld,btRigidBody* rigidBody, btRigidBody* ground,const btVector3& targetPos) {
    // ������ʱ����ײ��״�͸���
    btCollisionShape* shape = rigidBody->getCollisionShape();
    btDefaultMotionState tempMotionState(btTransform(btQuaternion(0, 0, 0, 1), targetPos));
    btRigidBody tempRigidBody(1.0, &tempMotionState, shape);

    // ������ʱ���岻�ص��������Ҫ��
    tempRigidBody.setRestitution(0.0f);

    // �����Զ���Ļص����󣬴���tempRigidBody��Ϊ����
    MyContactResultCallback contactCallback(&tempRigidBody);

    // ����ԭʼ���壨��������ײ�жϣ�
    contactCallback.setOriginalBody(rigidBody);
    contactCallback.set_ground(ground);
    // ʹ�� ContactTest ��������ײ����
    dynamicsWorld->contactTest(&tempRigidBody, contactCallback);

    // �����Ƿ�������ײ
    return contactCallback.hasCollision;
}
//��ת����
void Model::rotateObject(float angleInDegrees) {
    // ��ȡ��ǰ�任
    btTransform transform;
    rigidBody->getMotionState()->getWorldTransform(transform);

    // ��ǰ��ת
    btQuaternion currentRotation = transform.getRotation();

    // ������ת���� (�� Y ����ת)
    float angleInRadians = btRadians(angleInDegrees);
    btQuaternion deltaRotation(btVector3(0, 1, 0), angleInRadians);

    // �����µ���ת
    btQuaternion newRotation = deltaRotation * currentRotation;

    // Ӧ���µ���ת
    transform.setRotation(newRotation);

    // ���¸���ı任
    rigidBody->getMotionState()->setWorldTransform(transform);
}


//�ƶ�����
void Model::moveModel(btDiscreteDynamicsWorld* dynamicsWorld,float dx, float dy, float dz)
{
    // ��ȡ��ǰ����ı任
    btTransform transform;
    rigidBody->getMotionState()->getWorldTransform(transform);

    // ��ȡ��ǰ��λ�ú���ת
    btVector3 currentPos = transform.getOrigin();
    btQuaternion rotation = transform.getRotation();

    // ����ȫ���ƶ�����
    btVector3 localDirection(dx, dy, dz);
    btVector3 globalDirection = quatRotate(rotation, localDirection);

    // ����Ŀ��λ��
    btVector3 targetPos = currentPos + globalDirection;

    // ���Ŀ��λ���Ƿ�������������ײ
    if (!checkCollisionAt(dynamicsWorld,rigidBody,ground,targetPos)) {
        // ���û����ײ���������ƶ���Ŀ��λ��
        transform.setOrigin(targetPos);
        rigidBody->getMotionState()->setWorldTransform(transform);
        update_position();
    }
}
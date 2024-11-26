#include"Model.h"
#include <btBulletDynamicsCommon.h>

// 创建自定义的回调类
class MyContactResultCallback : public btCollisionWorld::ContactResultCallback {
public:
    bool hasCollision = false;
    btRigidBody* tempRigidBody;  // 保存目标刚体的指针
    btCollisionObject* originalBody;  // 保存原始物体的指针
    btCollisionObject* ground;
    MyContactResultCallback(btRigidBody* body) : tempRigidBody(body), originalBody(nullptr) {}

    // 设置原始物体（仅用于排除与自己碰撞）
    void setOriginalBody(btCollisionObject* body) {
        originalBody = body;
    }
    void set_ground(btCollisionObject* _ground)
    {
        ground = _ground;
    }

    btScalar addSingleResult(btManifoldPoint& point, const btCollisionObjectWrapper* colObj0, int partId0, int index0,
        const btCollisionObjectWrapper* colObj1, int partId1, int index1) override {
        // 确保排除与自身的碰撞
        if (colObj0->getCollisionObject() != ground && colObj1->getCollisionObject() != ground) {
            if (colObj0->getCollisionObject() != originalBody && colObj1->getCollisionObject() != originalBody) {
                if (colObj0->getCollisionObject() != originalBody && colObj1->getCollisionObject() != originalBody) {
                    // 检查碰撞对象是否为目标刚体
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
    case 0: // 球形
        collisionShape = new btSphereShape(dimensions.x()); // 使用 x 作为半径
        break;
    case 1: // 胶囊
        collisionShape = new btCapsuleShape(dimensions.x(), dimensions.y());
        // x 是半径, y 是长度（不包括两端半球）
        break;
    case 2: // 圆柱
        collisionShape = new btCylinderShape(dimensions);
        // dimensions 的 x/y/z 分别是半径和高度的方向
        break;
    case 3: // 盒子
        collisionShape = new btBoxShape(dimensions); // 盒子半边长
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
    // 获取四元数
    btQuaternion rotation = trans.getRotation();
    // 将四元数转换为欧拉角 (ZYX 顺序)
    btScalar yaw, pitch, roll; // Yaw (Z), Pitch (Y), Roll (X)
    rotation.getEulerZYX(yaw, pitch, roll);
    // 将欧拉角封装到 glm::vec3 中（需要转换为度数）
    glm::vec3 eulerAngles(glm::degrees(roll), glm::degrees(pitch), glm::degrees(yaw));
    // 设置旋转
    setRotation(eulerAngles);
    //printf("x:%f   y:%f   z:%f\n", trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
    //printf("x:%f   y:%f   z:%f\n", scale.x, scale.y, scale.z);
}
//键盘操作
void Model::key_callback(btDiscreteDynamicsWorld* dynamicsWorld, int key, int action, int mode)
{
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) {
        case GLFW_KEY_W: // 向前
            moveModel(dynamicsWorld, 0, 0, -1);
            break;
        case GLFW_KEY_D:
            moveModel(dynamicsWorld, 0, 0, 1);
            break;
        // 旋转
        case GLFW_KEY_Q: rotateObject(10); break;  // 逆时针
        case GLFW_KEY_E: rotateObject(-10); break; // 顺时针

        }
    }
}

//碰撞检测
bool checkCollisionAt(btDiscreteDynamicsWorld* dynamicsWorld,btRigidBody* rigidBody, btRigidBody* ground,const btVector3& targetPos) {
    // 创建临时的碰撞形状和刚体
    btCollisionShape* shape = rigidBody->getCollisionShape();
    btDefaultMotionState tempMotionState(btTransform(btQuaternion(0, 0, 0, 1), targetPos));
    btRigidBody tempRigidBody(1.0, &tempMotionState, shape);

    // 设置临时刚体不回弹（如果需要）
    tempRigidBody.setRestitution(0.0f);

    // 创建自定义的回调对象，传递tempRigidBody作为参数
    MyContactResultCallback contactCallback(&tempRigidBody);

    // 设置原始物体（不参与碰撞判断）
    contactCallback.setOriginalBody(rigidBody);
    contactCallback.set_ground(ground);
    // 使用 ContactTest 来进行碰撞测试
    dynamicsWorld->contactTest(&tempRigidBody, contactCallback);

    // 返回是否发生了碰撞
    return contactCallback.hasCollision;
}
//旋转操作
void Model::rotateObject(float angleInDegrees) {
    // 获取当前变换
    btTransform transform;
    rigidBody->getMotionState()->getWorldTransform(transform);

    // 当前旋转
    btQuaternion currentRotation = transform.getRotation();

    // 计算旋转增量 (绕 Y 轴旋转)
    float angleInRadians = btRadians(angleInDegrees);
    btQuaternion deltaRotation(btVector3(0, 1, 0), angleInRadians);

    // 更新新的旋转
    btQuaternion newRotation = deltaRotation * currentRotation;

    // 应用新的旋转
    transform.setRotation(newRotation);

    // 更新刚体的变换
    rigidBody->getMotionState()->setWorldTransform(transform);
}


//移动操作
void Model::moveModel(btDiscreteDynamicsWorld* dynamicsWorld,float dx, float dy, float dz)
{
    // 获取当前刚体的变换
    btTransform transform;
    rigidBody->getMotionState()->getWorldTransform(transform);

    // 获取当前的位置和旋转
    btVector3 currentPos = transform.getOrigin();
    btQuaternion rotation = transform.getRotation();

    // 计算全局移动方向
    btVector3 localDirection(dx, dy, dz);
    btVector3 globalDirection = quatRotate(rotation, localDirection);

    // 计算目标位置
    btVector3 targetPos = currentPos + globalDirection;

    // 检查目标位置是否与其他物体碰撞
    if (!checkCollisionAt(dynamicsWorld,rigidBody,ground,targetPos)) {
        // 如果没有碰撞，将物体移动到目标位置
        transform.setOrigin(targetPos);
        rigidBody->getMotionState()->setWorldTransform(transform);
        update_position();
    }
}
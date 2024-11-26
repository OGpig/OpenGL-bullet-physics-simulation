
#include<random>
#include <btBulletDynamicsCommon.h>
#include "Angel.h"
#include "TriMesh.h"
#include "Camera.h"
#include "MeshPainter.h"
#include"Ground.h"
#include"Model.h"
#include <vector>
#include <string>


// 设置随机数生成器
std::random_device rd;  // 随机数种子
std::mt19937 gen(rd()); // 随机数引擎
std::uniform_real_distribution<float> dist(-20.0f, 20.0f); // 均匀分布 [-20, 20]

int WIDTH = 800;
int HEIGHT = 800;

int mainWindow;//主窗口

float waterScale = 100.0f;

const float pos[8] = { 0.0,5.0,10.0,5.0,0.0,-5.0,-10.0,-5.0 };//位置角度坐标

int LUA = 0, RUA = 4, LUL = 4, RUL = 0;

//只在每次键盘事件后修改，防止重复渲染，加速程序运行速度
int pause_tag = 0;
int camera_tag = 0;
class MatrixStack {
    int		_index;
    int		_size;
    glm::mat4* _matrices;

public:
    MatrixStack(int numMatrices = 100) :_index(0), _size(numMatrices)
    {
        _matrices = new glm::mat4[numMatrices];
    }

    ~MatrixStack()
    {
        delete[]_matrices;
    }

    void push(const glm::mat4& m) {
        assert(_index + 1 < _size);
        _matrices[_index++] = m;
    }

    glm::mat4& pop() {
        assert(_index - 1 >= 0);
        _index--;
        return _matrices[_index];
    }
};


struct Robot
{
    float TORSO_HEIGHT = 0.6;
    float TORSO_WIDTH = 0.4;
    float UPPER_ARM_HEIGHT = 0.3;
    float LOWER_ARM_HEIGHT = 0.3;
    float UPPER_ARM_WIDTH = 0.2;
    float LOWER_ARM_WIDTH = 0.2;
    float UPPER_LEG_HEIGHT = 0.3;
    float LOWER_LEG_HEIGHT = 0.3;
    float UPPER_LEG_WIDTH = 0.2;
    float LOWER_LEG_WIDTH = 0.2;
    float HEAD_HEIGHT = 0.4;
    float HEAD_WIDTH = 0.4;
    // 关节角和菜单选项值
    enum {
        Torso,			// 躯干
        Head,			// 头部
        RightUpperArm,	// 右大臂
        RightLowerArm,	// 右小臂
        LeftUpperArm,	// 左大臂
        LeftLowerArm,	// 左小臂
        RightUpperLeg,	// 右大腿
        RightLowerLeg,	// 右小腿
        LeftUpperLeg,	// 左大腿
        LeftLowerLeg,	// 左小腿
    };

    // 关节角大小
    GLfloat theta[10] = {
        0.0,    // Torso
        0.0,    // Head
        0.0,    // RightUpperArm
        0.0,    // RightLowerArm
        0.0,    // LeftUpperArm
        0.0,    // LeftLowerArm
        0.0,    // RightUpperLeg
        0.0,    // RightLowerLeg
        0.0,    // LeftUpperLeg
        0.0     // LeftLowerLeg
    };
    // 清零 theta 数组的函数
    void resetTheta() {
        for (int i = 0; i < 10; ++i) {
            theta[i] = 0.0;
        }
    }
};

Robot robot;


//史蒂夫的位置信息
glm::vec3 pos_vec = glm::vec3(0.0, 0.30 * robot.TORSO_HEIGHT, 0.0);

TriMesh* Torso = new TriMesh();
TriMesh* Head = new TriMesh();
TriMesh* RightUpperArm = new TriMesh();
TriMesh* RightLowerArm = new TriMesh();
TriMesh* LeftUpperArm = new TriMesh();
TriMesh* LeftLowerArm = new TriMesh();
TriMesh* RightUpperLeg = new TriMesh();
TriMesh* RightLowerLeg = new TriMesh();
TriMesh* LeftUpperLeg = new TriMesh();
TriMesh* LeftLowerLeg = new TriMesh();


// 被选中的物体
int Selected_mesh = robot.Torso;


Camera* camera = new Camera();
Camera* camera1 = new Camera();
Camera* cur_camera = new Camera();
Light* light = new Light();
MeshPainter* painter = new MeshPainter();

std::string vshader, fshader;
// 这个用来回收和删除我们创建的物体对象
std::vector<TriMesh*> meshList;

TriMesh* skybox1 = new TriMesh();
TriMesh* skybox2 = new TriMesh();
TriMesh* skybox3 = new TriMesh();
TriMesh* skybox4 = new TriMesh();
TriMesh* skybox5 = new TriMesh();
TriMesh* skybox6 = new TriMesh();

//树木
TriMesh* Tree[10];

Fluid* water = new Fluid(0.1f);    //水面
float Scale = 10.0f;

Ground* ground = new Ground(); //地面
Model* cube = new Model();     //立方体
Model* cube2 = new Model();


void settexure(TriMesh* mesh, glm::vec4 mat_ambient, glm::vec4 mat_diffuse, glm::vec4 mat_specular, float shine)
{
    // 设置材质
    mesh->setAmbient(mat_ambient); // 环境光
    mesh->setDiffuse(mat_diffuse); // 漫反射
    mesh->setSpecular(mat_specular); // 镜面反射
    mesh->setShininess(shine); //高光系数
}


//物理世界
btDiscreteDynamicsWorld* dynamicsWorld;
void init()
{
    // 读取着色器并使用
#ifdef __APPLE__	// for MacOS
    vshader = "shaders/vshader_mac.glsl";
    fshader = "shaders/fshader_mac.glsl";
#else				// for Windows
    vshader = "shaders/vshader_win.glsl";
    fshader = "shaders/fshader_win.glsl";
#endif
    // 初始化物理世界
    btBroadphaseInterface* broadphase = new btDbvtBroadphase();
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
    glm::vec4 mat = glm::vec4(1, 1, 1, 1);
    // 创建重力
    dynamicsWorld->setGravity(btVector3(0, -9.81, 0));

    // 设置光源位置
    light->setTranslation(glm::vec3(0.0, waterScale / 2, -waterScale / 2));
    light->setAmbient(glm::vec4(1.0, 1.0, 1.0, 1.0)); // 环境光
    light->setDiffuse(glm::vec4(1.0, 1.0, 1.0, 1.0)); // 漫反射
    light->setSpecular(glm::vec4(1.0, 1.0, 1.0, 1.0)); // 镜面反射
    light->setAttenuation(1.0, 0.45, 0.075); // 衰减系数

    glm::vec4 mat_ambient = { 1, 1, 1,1.0f };
    glm::vec4 mat_diffuse = { 1, 1, 1, 1.0f };
    glm::vec4 mat_specular = { 0.5f, 0.5f, 0.5f, 1.0f };
    float shine = 20.0f;
    // 保持变换矩阵的栈
    MatrixStack mstack;
    glm::mat4 modelMatrix = glm::mat4(1.0);

    // 躯干（这里我们希望机器人的躯干只绕Y轴旋转，所以只计算了RotateY）
    modelMatrix = glm::translate(modelMatrix, pos_vec);
    modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.Torso]), glm::vec3(0.0, 1.0, 0.0));
    //设置材质
    settexure(Torso, mat_ambient, mat_diffuse, mat_specular, shine);
    Torso->generateCube();
    Torso->setTranslation(glm::vec3(0.0, 0.5 * robot.TORSO_HEIGHT, 0.0));
    Torso->setRotation(glm::vec3(0.0f, 0.0f, 0.0f));
    Torso->setScale(glm::vec3(robot.TORSO_WIDTH, robot.TORSO_HEIGHT, robot.TORSO_WIDTH));
    painter->addMesh(Torso, "Torso", "./assets/cube/wall.jpg", vshader, fshader, 3, modelMatrix); 	// 指定纹理与着色器
    meshList.push_back(Torso);
    mstack.push(modelMatrix); // 保存躯干变换矩阵


    modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0, 0.65 * robot.TORSO_HEIGHT, 0.0));
    modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.Head]), glm::vec3(0.0, 1.0, 0.0));
    settexure(Head, mat_ambient, mat_diffuse, mat_specular, shine);
    Head->readObj("./assets/cube/cube2.obj");
    Head->setTranslation(glm::vec3(0.0, 0.5 * robot.HEAD_HEIGHT, 0.0));
    Head->setRotation(glm::vec3(0.0f, 0.0f, 0.0f));
    Head->setScale(glm::vec3(robot.HEAD_WIDTH, robot.HEAD_HEIGHT, robot.HEAD_WIDTH));
    camera1->init_eyes(Head);
    painter->addMesh(Head, "Head", "./assets/cube/steve.jpg", vshader, fshader, 3, modelMatrix); 	// 指定纹理与着色器
    meshList.push_back(Head);
    modelMatrix = mstack.pop(); // 恢复躯干变换矩阵

    mstack.push(modelMatrix);   // 保存躯干变换矩阵
    // 左大臂（这里我们希望机器人的左大臂只绕Z轴旋转，所以只计算了RotateZ，后面同理）
    modelMatrix = glm::translate(modelMatrix, glm::vec3(-0.45 * robot.TORSO_WIDTH, 0.9 * robot.TORSO_HEIGHT, 0.0));
    modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.LeftUpperArm]), glm::vec3(1.0, 0.0, 0.0));
    settexure(LeftUpperArm, mat_ambient, mat_diffuse, mat_specular, shine);
    LeftUpperArm->generateCube();
    LeftUpperArm->setTranslation(glm::vec3(0.0, -0.5 * robot.UPPER_ARM_HEIGHT, 0.0));
    LeftUpperArm->setRotation(glm::vec3(0.0f, 0.0f, 0.0f));
    LeftUpperArm->setScale(glm::vec3(robot.UPPER_ARM_WIDTH, robot.UPPER_ARM_HEIGHT, robot.UPPER_ARM_WIDTH));
    painter->addMesh(LeftUpperArm, "LeftUpperArm", "./assets/cube/wall.jpg", vshader, fshader, 3, modelMatrix); 	// 指定纹理与着色器
    meshList.push_back(LeftUpperArm);

    modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0, -robot.UPPER_ARM_HEIGHT, 0.0));
    modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.LeftLowerArm]), glm::vec3(1.0, 0.0, 0.0));
    settexure(LeftLowerArm, mat_ambient, mat_diffuse, mat_specular, shine);
    LeftLowerArm->generateCube();
    LeftLowerArm->setTranslation(glm::vec3(0.0, 0.0f, 0.0));
    LeftLowerArm->setRotation(glm::vec3(0.0f, 0.0f, 0.0f));
    LeftLowerArm->setScale(glm::vec3(robot.LOWER_ARM_WIDTH, robot.LOWER_ARM_HEIGHT, robot.LOWER_ARM_WIDTH));
    painter->addMesh(LeftLowerArm, "LeftLowerArm", "./assets/cube/hand.jpg", vshader, fshader, 3, modelMatrix); 	// 指定纹理与着色器
    meshList.push_back(LeftLowerArm);

    modelMatrix = mstack.pop(); // 恢复躯干变换矩阵

// =========== 右臂 ===========
    mstack.push(modelMatrix);   // 保存躯干变换矩阵
    // @TODO: 右大臂
    modelMatrix = glm::translate(modelMatrix, glm::vec3(0.45 * robot.TORSO_WIDTH, 0.9 * robot.TORSO_HEIGHT, 0.0));
    modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.RightUpperArm]), glm::vec3(1.0, 0.0, 0.0));
    settexure(RightUpperArm, mat_ambient, mat_diffuse, mat_specular, shine);
    RightUpperArm->generateCube();
    RightUpperArm->setTranslation(glm::vec3(0.0, -0.5 * robot.UPPER_ARM_HEIGHT, 0.0));
    RightUpperArm->setRotation(glm::vec3(0.0f, 0.0f, 0.0f));
    RightUpperArm->setScale(glm::vec3(robot.UPPER_ARM_WIDTH, robot.UPPER_ARM_HEIGHT, robot.UPPER_ARM_WIDTH));
    painter->addMesh(RightUpperArm, "RightUpperArm", "./assets/cube/wall.jpg", vshader, fshader, 3, modelMatrix); 	// 指定纹理与着色器
    meshList.push_back(RightUpperArm);

    modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0, -robot.UPPER_ARM_HEIGHT, 0.0));
    modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.RightLowerArm]), glm::vec3(1.0, 0.0, 0.0));
    settexure(RightLowerArm, mat_ambient, mat_diffuse, mat_specular, shine);
    RightLowerArm->generateCube();
    RightLowerArm->setTranslation(glm::vec3(0.0, 0.0, 0.0));
    RightLowerArm->setRotation(glm::vec3(0.0f, 0.0f, 0.0f));
    RightLowerArm->setScale(glm::vec3(robot.LOWER_ARM_WIDTH, robot.LOWER_ARM_HEIGHT, robot.LOWER_ARM_WIDTH));
    painter->addMesh(RightLowerArm, "RightLowerArm", "./assets/cube/hand.jpg", vshader, fshader, 3, modelMatrix); 	// 指定纹理与着色器
    meshList.push_back(RightLowerArm);

    modelMatrix = mstack.pop(); // 恢复躯干变换矩阵

    // =========== 左腿 ===========
    mstack.push(modelMatrix);   // 保存躯干变换矩阵
    modelMatrix = glm::translate(modelMatrix, glm::vec3(-0.15 * robot.TORSO_WIDTH, 0.1 * robot.TORSO_HEIGHT, 0.0));
    modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.LeftUpperLeg]), glm::vec3(1.0, 0.0, 0.0));
    settexure(LeftUpperLeg, mat_ambient, mat_diffuse, mat_specular, shine);
    LeftUpperLeg->generateCube();
    LeftUpperLeg->setTranslation(glm::vec3(0.0, 0.0f, 0.0));
    LeftUpperLeg->setRotation(glm::vec3(0.0f, 0.0f, 0.0f));
    LeftUpperLeg->setScale(glm::vec3(robot.UPPER_LEG_WIDTH, robot.UPPER_LEG_HEIGHT, robot.UPPER_LEG_WIDTH));
    painter->addMesh(LeftUpperLeg, "LeftUpperLeg", "./assets/cube/wall.jpg", vshader, fshader, 3, modelMatrix); 	// 指定纹理与着色器
    meshList.push_back(LeftUpperLeg);

    modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0, -0.5 * robot.UPPER_LEG_HEIGHT, 0.0));
    modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.LeftLowerLeg]), glm::vec3(1.0, 0.0, 0.0));
    settexure(LeftLowerLeg, mat_ambient, mat_diffuse, mat_specular, shine);
    LeftLowerLeg->generateCube();
    LeftLowerLeg->setTranslation(glm::vec3(0.0, 0.0f, 0.0));
    LeftLowerLeg->setRotation(glm::vec3(0.0f, 0.0f, 0.0f));
    LeftLowerLeg->setScale(glm::vec3(robot.LOWER_LEG_WIDTH, robot.LOWER_LEG_HEIGHT, robot.LOWER_LEG_WIDTH));
    painter->addMesh(LeftLowerLeg, "LeftLowerLeg", "./assets/cube/hand.jpg", vshader, fshader, 3, modelMatrix); 	// 指定纹理与着色器
    meshList.push_back(LeftLowerLeg);
    modelMatrix = mstack.pop(); // 恢复躯干变换矩阵

    // =========== 右腿 ===========
    mstack.push(modelMatrix);   // 保存躯干变换矩阵
    // @TODO: 右大腿
    modelMatrix = glm::translate(modelMatrix, glm::vec3(0.15 * robot.TORSO_WIDTH, 0.1 * robot.TORSO_HEIGHT, 0.0));
    modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.RightUpperLeg]), glm::vec3(1.0, 0.0, 0.0));
    settexure(RightUpperLeg, mat_ambient, mat_diffuse, mat_specular, shine);
    RightUpperLeg->generateCube();
    RightUpperLeg->setTranslation(glm::vec3(0.0, 0.0f, 0.0));
    RightUpperLeg->setRotation(glm::vec3(0.0f, 0.0f, 0.0f));
    RightUpperLeg->setScale(glm::vec3(robot.UPPER_LEG_WIDTH, robot.UPPER_LEG_HEIGHT, robot.UPPER_LEG_WIDTH));
    painter->addMesh(RightUpperLeg, "RightUpperLeg", "./assets/cube/wall.jpg", vshader, fshader, 3, modelMatrix); 	// 指定纹理与着色器
    meshList.push_back(RightUpperLeg);

    modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0, -0.5 * robot.UPPER_LEG_HEIGHT, 0.0));
    modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.RightLowerLeg]), glm::vec3(1.0, 0.0, 0.0));
    settexure(RightLowerLeg, mat_ambient, mat_diffuse, mat_specular, shine);
    RightLowerLeg->generateCube();
    RightLowerLeg->setTranslation(glm::vec3(0.0, 0.0f, 0.0));
    RightLowerLeg->setRotation(glm::vec3(0.0f, 0.0f, 0.0f));
    RightLowerLeg->setScale(glm::vec3(robot.LOWER_LEG_WIDTH, robot.LOWER_LEG_HEIGHT, robot.LOWER_LEG_WIDTH));
    painter->addMesh(RightLowerLeg, "RightLowerLeg", "./assets/cube/hand.jpg", vshader, fshader, 3, modelMatrix); 	// 指定纹理与着色器
    meshList.push_back(RightLowerLeg);
    modelMatrix = mstack.pop(); // 恢复躯干变换矩阵


    ground->generateSquare(glm::vec3(0.5, 0.5, 0.8));
    ground->setTranslation(glm::vec3(0.0, -0.01, 0.0));
    ground->setRotation(glm::vec3(-90, 0, 0));
    ground->setScale(glm::vec3(100.0, 100.0, 1.0));
    ground->create();
    dynamicsWorld->addRigidBody(ground->rigidBody);

    painter->addMesh(ground, "mesh_ground", "./assets/mountain/ground.jpg", vshader, fshader, 1, glm::mat4(1.0f));
    meshList.push_back(ground);


    cube->generateCube();
    settexure(cube, mat_ambient, mat_diffuse, mat_specular, shine);
    cube->setTranslation(glm::vec3(4.0f, 10.0f, 4.0f));
    cube->setRotation(glm::vec3(-90, 0, 0));
    cube->setScale(glm::vec3(4, 4, 4));
    cube->create(1.0f, btVector3(0, 0, 0), 0, btVector3(1, 1, 1));
    dynamicsWorld->addRigidBody(cube->rigidBody);
    painter->addMesh(cube, "cube", "./assets/grass.jpg", vshader, fshader, 3, glm::mat4(1.0f));
    meshList.push_back(cube);
    cube->set_ground(ground->rigidBody);

    cube2->generateCube();
    settexure(cube2, mat_ambient, mat_diffuse, mat_specular, shine);
    cube2->setTranslation(glm::vec3(0.0f, 1.0f, 6.0f));
    cube2->setRotation(glm::vec3(-90, 0, 0.0f));
    cube2->setScale(glm::vec3(4, 4, 4));
    cube2->create(0.0f, btVector3(0, 0, 0), 0, btVector3(1, 1, 1));
    dynamicsWorld->addRigidBody(cube2->rigidBody);
    painter->addMesh(cube2, "cube2", "./assets/cube/wall.jpg", vshader, fshader, 3, glm::mat4(1.0f));
    meshList.push_back(cube2);

    // 水面
    water->setAmbient(glm::vec4(0.5f, 0.6f, 0.8f, 1.0f)); // 环境光
    water->setDiffuse(glm::vec4(0.8f, 0.9f, 1.0f, 1.0f)); // 漫反射
    water->setSpecular(glm::vec4(1.0f, 1.0f, 1.0f, 1.0f)); // 镜面反射
    water->setShininess(100.0f); //高光系数
    water->generateSurface(30, glm::vec3(0.0, 0.0, 0.0), 0.03f, 0.0f, 0.05f);
    water->setTranslation(glm::vec3(1, 0.35, -2));
    water->setRotation(glm::vec3(0, 0, 0));
    water->setScale(glm::vec3(3.0, 1.0, 3.0));

    painter->addMesh(water, "mesh_water", "./assets/water.jpg", vshader, fshader, 3, glm::mat4(1.0f));
    meshList.push_back(water);

    //创建树
    for (int i = 0; i < 10; i++)
    {
        Tree[i] = new TriMesh();
        settexure(Tree[i], mat_ambient, mat_diffuse, mat_specular, shine);
        Tree[i]->readObj("./assets/tree/1.obj");
        // 设置随机位置
        float randomX = dist(gen);
        float randomZ = dist(gen);
        Tree[i]->setTranslation(glm::vec3(randomX, 4.0f, randomZ));
        Tree[i]->setRotation(glm::vec3(0, 0, 0.0f));
        Tree[i]->setScale(glm::vec3(10, 10, 10));
        // 动态生成名字
        std::ostringstream treeName;
        treeName << "Tree_" << i; // 例如：Tree_0, Tree_1, ...
        painter->addMesh(Tree[i], treeName.str(), "./assets/tree/tree.png", vshader, fshader, 3, glm::mat4(1.0f));
        meshList.push_back(Tree[i]);
    }
    //天空盒
    skybox1->generateSquare(glm::vec3(0.0, 0.0, 0.0));

    skybox1->setTranslation(glm::vec3(0.0, 0.0, -waterScale / 2));
    skybox1->setRotation(glm::vec3(0.0, 0.0, 0.0));
    skybox1->setScale(glm::vec3(waterScale * sqrt(2.0), waterScale * sqrt(2.0), waterScale * sqrt(2.0)));

    painter->addMesh(skybox1, "skybox1", "./assets/skybox6.jpg", vshader, fshader, 1, glm::mat4(1.0f));
    meshList.push_back(skybox1);

    skybox2->generateSquare(glm::vec3(0.0, 0.0, 0.0));

    skybox2->setTranslation(glm::vec3(0.0, 0.0, waterScale / 2));
    skybox2->setRotation(glm::vec3(0.0, 180.0, 0.0));
    skybox2->setScale(glm::vec3(waterScale * sqrt(2.0), waterScale * sqrt(2.0), waterScale * sqrt(2.0)));

    painter->addMesh(skybox2, "skybox2", "./assets/skybox8.jpg", vshader, fshader, 1, glm::mat4(1.0f));
    meshList.push_back(skybox2);

    skybox3->generateSquare(glm::vec3(0.0, 0.0, 0.0));

    skybox3->setTranslation(glm::vec3(-waterScale / 2, 0.0, 0.0));
    skybox3->setRotation(glm::vec3(0.0, 90.0, 0.0));
    skybox3->setScale(glm::vec3(waterScale * sqrt(2.0), waterScale * sqrt(2.0), waterScale * sqrt(2.0)));

    painter->addMesh(skybox3, "skybox3", "./assets/skybox5.jpg", vshader, fshader, 1, glm::mat4(1.0f));
    meshList.push_back(skybox3);

    skybox4->generateSquare(glm::vec3(0.0, 0.0, 0.0));

    skybox4->setTranslation(glm::vec3(waterScale / 2, 0.0, 0.0));
    skybox4->setRotation(glm::vec3(0.0, -90.0, 0.0));
    skybox4->setScale(glm::vec3(waterScale * sqrt(2.0), waterScale * sqrt(2.0), waterScale * sqrt(2.0)));

    painter->addMesh(skybox4, "skybox4", "./assets/skybox7.jpg", vshader, fshader, 1, glm::mat4(1.0f));
    meshList.push_back(skybox4);

    skybox5->generateSquare(glm::vec3(0.0, 0.0, 0.0));

    skybox5->setTranslation(glm::vec3(0.0, waterScale / 2, 0.0));
    skybox5->setRotation(glm::vec3(90.0, 0.0, 0.0));
    skybox5->setScale(glm::vec3(waterScale * sqrt(2.0), waterScale * sqrt(2.0), waterScale * sqrt(2.0)));

    painter->addMesh(skybox5, "skybox5", "./assets/skybox2.jpg", vshader, fshader, 1, glm::mat4(1.0f));
    meshList.push_back(skybox5);

    skybox6->generateSquare(glm::vec3(0.0, 0.0, 0.0));

    skybox6->setTranslation(glm::vec3(0.0, -waterScale / 2, 0.0));
    skybox6->setRotation(glm::vec3(-90.0, 0.0, 0.0));
    skybox6->setScale(glm::vec3(waterScale * sqrt(2.0), waterScale * sqrt(2.0), waterScale * sqrt(2.0)));

    painter->addMesh(skybox6, "skybox6", "./assets/skybox10.jpg", vshader, fshader, 1, glm::mat4(1.0f));
    meshList.push_back(skybox6);
    glClearColor(1.0, 1.0, 1.0, 1.0);
    // glClearColor(0.0, 0.0, 0.0, 1.0);
}
//背景跟随相机
void fix_backgroud()
{
    painter->replaceMesh1(skybox1, "skybox1", "./assets/skybox/mars_negx.jpg", vshader, fshader, 1, glm::mat4(1.0f));
    painter->replaceMesh1(skybox2, "skybox2", "./assets/skybox/mars_posx.jpg", vshader, fshader, 1, glm::mat4(1.0f));
    painter->replaceMesh1(skybox3, "skybox3", "./assets/skybox/mars_negz.jpg", vshader, fshader, 1, glm::mat4(1.0f));
    painter->replaceMesh1(skybox4, "skybox4", "./assets/skybox/mars_posz.jpg", vshader, fshader, 1, glm::mat4(1.0f));
    painter->replaceMesh1(skybox5, "skybox5", "./assets/skybox/mars_posy.jpg", vshader, fshader, 1, glm::mat4(1.0f));
    painter->replaceMesh1(skybox6, "skybox6", "./assets/skybox/mars_negy.jpg", vshader, fshader, 1, glm::mat4(1.0f));
}
void update_steve()
{
    MatrixStack mstack;
    glm::mat4 modelMatrix = glm::mat4(1.0f);
    // 躯干（这里我们希望机器人的躯干只绕Y轴旋转，所以只计算了RotateY）
    modelMatrix = glm::translate(modelMatrix, pos_vec);
    modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.Torso]), glm::vec3(0.0, 1.0, 0.0));
    painter->replaceMesh(Torso, "Torso", "./assets/cube/wall.jpg", vshader, fshader, 3, modelMatrix);
    mstack.push(modelMatrix); // 保存躯干变换矩阵

    camera1->init_eyes(Head);
    modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0, 0.65 * robot.TORSO_HEIGHT, 0.0));
    modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.Head]), glm::vec3(0.0, 1.0, 0.0));
    painter->replaceMesh(Head, "Head", "./assets/cube/steve.jpg", vshader, fshader, 3, modelMatrix); 	// 指定纹理与着色器
    meshList.push_back(Head);
    camera1->follow_body(Head, modelMatrix);
    modelMatrix = mstack.pop(); // 恢复躯干变换矩阵

    mstack.push(modelMatrix);   // 保存躯干变换矩阵
    // 左大臂（这里我们希望机器人的左大臂只绕Z轴旋转，所以只计算了RotateZ，后面同理）
    modelMatrix = glm::translate(modelMatrix, glm::vec3(-0.45 * robot.TORSO_WIDTH, 0.9 * robot.TORSO_HEIGHT, 0.0));
    modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.LeftUpperArm]), glm::vec3(1.0, 0.0, 0.0));
    painter->replaceMesh(LeftUpperArm, "LeftUpperArm", "./assets/cube/wall.jpg", vshader, fshader, 3, modelMatrix); 	// 指定纹理与着色器
    meshList.push_back(LeftUpperArm);

    modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0, -robot.UPPER_ARM_HEIGHT, 0.0));
    modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.LeftLowerArm]), glm::vec3(1.0, 0.0, 0.0));
    painter->replaceMesh(LeftLowerArm, "LeftLowerArm", "./assets/cube/hand.jpg", vshader, fshader, 3, modelMatrix); 	// 指定纹理与着色器
    meshList.push_back(LeftLowerArm);

    modelMatrix = mstack.pop(); // 恢复躯干变换矩阵

// =========== 右臂 ===========
    mstack.push(modelMatrix);   // 保存躯干变换矩阵
    // @TODO: 右大臂
    modelMatrix = glm::translate(modelMatrix, glm::vec3(0.45 * robot.TORSO_WIDTH, 0.9 * robot.TORSO_HEIGHT, 0.0));
    modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.RightUpperArm]), glm::vec3(1.0, .0, 0.0));
    painter->replaceMesh(RightUpperArm, "RightUpperArm", "./assets/cube/wall.jpg", vshader, fshader, 3, modelMatrix); 	// 指定纹理与着色器
    meshList.push_back(RightUpperArm);

    modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0, -robot.UPPER_ARM_HEIGHT, 0.0));
    modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.RightLowerArm]), glm::vec3(1.0, 0.0, 0.0));
    painter->replaceMesh(RightLowerArm, "RightLowerArm", "./assets/cube/hand.jpg", vshader, fshader, 3, modelMatrix); 	// 指定纹理与着色器
    meshList.push_back(RightLowerArm);

    modelMatrix = mstack.pop(); // 恢复躯干变换矩阵

    // =========== 左腿 ===========
    mstack.push(modelMatrix);   // 保存躯干变换矩阵
    modelMatrix = glm::translate(modelMatrix, glm::vec3(-0.15 * robot.TORSO_WIDTH, 0.1 * robot.TORSO_HEIGHT, 0.0));
    modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.LeftUpperLeg]), glm::vec3(1.0, 0.0, 0.0));
    painter->replaceMesh(LeftUpperLeg, "LeftUpperLeg", "./assets/cube/wall.jpg", vshader, fshader, 3, modelMatrix); 	// 指定纹理与着色器
    meshList.push_back(LeftUpperLeg);

    modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0, -0.5 * robot.UPPER_LEG_HEIGHT, 0.0));
    modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.LeftLowerLeg]), glm::vec3(1.0, 0.0, 0.0));
    painter->replaceMesh(LeftLowerLeg, "LeftLowerLeg", "./assets/cube/hand.jpg", vshader, fshader, 3, modelMatrix); 	// 指定纹理与着色器
    meshList.push_back(LeftLowerLeg);
    modelMatrix = mstack.pop(); // 恢复躯干变换矩阵

    // =========== 右腿 ===========
    mstack.push(modelMatrix);   // 保存躯干变换矩阵
    // @TODO: 右大腿
    modelMatrix = glm::translate(modelMatrix, glm::vec3(0.15 * robot.TORSO_WIDTH, 0.1 * robot.TORSO_HEIGHT, 0.0));
    modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.RightUpperLeg]), glm::vec3(1.0, 0.0, 0.0));
    painter->replaceMesh(RightUpperLeg, "RightUpperLeg", "./assets/cube/wall.jpg", vshader, fshader, 3, modelMatrix); 	// 指定纹理与着色器
    meshList.push_back(RightUpperLeg);

    modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0, -0.5 * robot.UPPER_LEG_HEIGHT, 0.0));
    modelMatrix = glm::rotate(modelMatrix, glm::radians(robot.theta[robot.RightLowerLeg]), glm::vec3(1.0, 0.0, 0.0));
    painter->replaceMesh(RightLowerLeg, "RightLowerLeg", "./assets/cube/hand.jpg", vshader, fshader, 3, modelMatrix); 	// 指定纹理与着色器
    meshList.push_back(RightLowerLeg);
    modelMatrix = mstack.pop(); // 恢复躯干变换矩阵
}
void display()
{
    // #ifdef __APPLE__ // 解决 macOS 10.15 显示画面缩小问题
    // 	glViewport(0, 0, WIDTH * 2, HEIGHT * 2);
    // #endif

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    if (pause_tag) {
        update_steve();
        pause_tag = 0;
    }
    if (camera_tag == 0)
        painter->drawMeshes(light, camera);
    else
        painter->drawMeshes(light, camera1);

    glDepthMask(GL_TRUE);

    //glutSwapBuffers();
}


void printHelp()
{
    std::cout << "================================================" << std::endl;
    std::cout << "Use mouse to controll the light position (drag)." << std::endl;
    std::cout << "================================================" << std::endl << std::endl;

    std::cout << "Keyboard Usage" << std::endl;
    std::cout <<
        "[Window]" << std::endl <<
        "ESC:		Exit" << std::endl <<
        "h:		Print help message" << std::endl <<

        std::endl <<
        "[Camera]" << std::endl <<
        "SPACE:		Reset camera parameters" << std::endl <<
        "u/U:		Increase/Decrease the rotate angle" << std::endl <<
        "i/I:		Increase/Decrease the up angle" << std::endl <<
        "o/O:		Increase/Decrease the light_x" << std::endl <<
        "j/J:		Increase/Decrease the light_y" << std::endl <<//可能对应不上
        "k/K:		Increase/Decrease the light_z" << std::endl <<
        "l/L:		Increase/Decrease the camera radius" << std::endl <<
        "  n: 		open or close ortho" << std::endl <<
        "  v:	    change the skybox" << std::endl <<
        "0~9:		select the body part" << std::endl <<
        "  a:		Increase body part theta" << std::endl <<
        "  s:		Decrease the camera radius" << std::endl <<
        " up:		move steve front" << std::endl <<
        "down:	   move steve back" << std::endl <<
        "  q:		rotate cube" << std::endl <<
        "  e:		rotate cube" << std::endl <<
        "  w:		go cube" << std::endl <<
        "  d:		back cube" << std::endl <<
        "space:	   reset other" << std::endl << 
        "z  :	    change perspective" << std::endl << std::endl;

}

//史蒂夫的行走动作
void move_steve()
{
    LUA = (++LUA) % 8;//左手
    RUA = (++RUA) % 8;//右手
    LUL = (++LUL) % 8;//左腿
    RUL = (++RUL) % 8;//右腿
    robot.theta[robot.LeftUpperArm] = pos[LUA];
    robot.theta[robot.RightUpperArm] = pos[RUA];
    robot.theta[robot.LeftUpperLeg] = pos[LUL];
    robot.theta[robot.RightUpperLeg] = pos[RUL];

}

// 键盘响应函数
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode)
{
    float tmp;
    glm::vec4 ambient;
    if (action == GLFW_PRESS) {
        switch (key)
        {
        case GLFW_KEY_ESCAPE: exit(EXIT_SUCCESS); break;
            //切换天空盒
        case GLFW_KEY_V:fix_backgroud(); break;
            //steve前进
        case GLFW_KEY_UP:pos_vec += glm::vec3(1.0f * sin(robot.theta[robot.Torso] / 180 * M_PI), 0.0f, 1.0f * cos(robot.theta[robot.Torso] / 180 * M_PI)); break;
            //steve后退
        case GLFW_KEY_DOWN: pos_vec -= glm::vec3(1.0f * sin(robot.theta[robot.Torso] / 180 * M_PI), 0.0f, 1.0f * cos(robot.theta[robot.Torso] / 180 * M_PI)); break;
        case GLFW_KEY_R:robot.resetTheta(); pos_vec = glm::vec3(0.0, 0.30 * robot.TORSO_HEIGHT, 0.0); pause_tag = 1; break;//动作清零
        //case GLFW_KEY_Q:move_steve(); pause_tag = 1; break;
        case GLFW_KEY_H: printHelp(); break;
        case GLFW_KEY_Z: camera_tag = 1 - camera_tag; break;
        case GLFW_KEY_1:Selected_mesh = robot.Torso; break;
        case GLFW_KEY_2:Selected_mesh = robot.Head; break;
        case GLFW_KEY_3:Selected_mesh = robot.LeftUpperArm; break;
        case GLFW_KEY_4:Selected_mesh = robot.LeftLowerArm; break;
        case GLFW_KEY_5:Selected_mesh = robot.RightUpperArm; break;
        case GLFW_KEY_6:Selected_mesh = robot.RightLowerArm; break;
        case GLFW_KEY_7:Selected_mesh = robot.LeftUpperLeg; break;
        case GLFW_KEY_8:Selected_mesh = robot.LeftLowerLeg; break;
        case GLFW_KEY_9:Selected_mesh = robot.RightUpperLeg; break;
        case GLFW_KEY_0:Selected_mesh = robot.RightLowerLeg; break;
        case GLFW_KEY_A:
            robot.theta[Selected_mesh] += 5.0;
            if (robot.theta[Selected_mesh] > 360.0)
                robot.theta[Selected_mesh] -= 360.0;
            pause_tag = 1;
            break;
        case GLFW_KEY_S:
            robot.theta[Selected_mesh] -= 5.0;
            if (robot.theta[Selected_mesh] < 0.0)
                robot.theta[Selected_mesh] += 360.0;
            pause_tag = 1;
            break;
        default:
            camera->keyboard(key, action, mode);
            light->keyboard(key, action, mode);
            cube->key_callback(dynamicsWorld, key, action, mode);
            break;
        }
    }
}

void cleanData() {
    // 释放内存

    delete camera;
    camera = NULL;

    delete light;
    light = NULL;

    painter->cleanMeshes();

    delete painter;
    painter = NULL;

    for (int i = 0; i < meshList.size(); i++) {
        delete meshList[i];
    }
    meshList.clear();
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height);

int main(int argc, char** argv)
{
    // 初始化GLFW库，必须是应用程序调用的第一个GLFW函数
    glfwInit();

    // 配置GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // 配置窗口属性
    GLFWwindow* window = glfwCreateWindow(800, 800, "2022150087赵晨阳期末大作业", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, key_callback);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // 调用任何OpenGL的函数之前初始化GLAD
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    // Init mesh, shaders, buffer
    init();
    // 输出帮助信息
    // 启用深度测试
    glEnable(GL_DEPTH_TEST);
    int t = 0;
    printHelp();
    while (!glfwWindowShouldClose(window))
    {
        if (t > 6)
        {
            t = 0;
            water->updateSurfacePosition();
            painter->replaceMesh1(water, "mesh_water", "./assets/water.jpg", vshader, fshader, 3, glm::mat4(1.0f));
            move_steve();
            pause_tag = 1;
        }
        t++;
        dynamicsWorld->stepSimulation(1.f / 60.f, 10);
        cube->update_position();
        painter->replaceMesh(cube, "cube", "./assets/grass.jpg", vshader, fshader, 3, glm::mat4(1.0f));
        //dynamicsWorld->updateSingleAabb(cube->rigidBody);
        display();
        //reshape();

        // 交换颜色缓冲 以及 检查有没有触发什么事件（比如键盘输入、鼠标移动等）
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    cleanData();


    return 0;
}

// 每当窗口改变大小，GLFW会调用这个函数并填充相应的参数供你处理。
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

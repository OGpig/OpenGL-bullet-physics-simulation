#include <btBulletDynamicsCommon.h>
#include "Angel.h"
#include "TriMesh.h"
#include "Camera.h"
#include "MeshPainter.h"
#include"Ground.h"
#include"Model.h"
#include <vector>
#include <string>

int WIDTH = 800;
int HEIGHT = 800;

int mainWindow;//主窗口

float waterScale = 100.0f;

Camera* camera = new Camera();
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

Ground* ground = new Ground(); //地面
Model* cube = new Model();     //立方体

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



	ground->generateSquare(glm::vec3(0.5, 0.5, 0.8));
	ground->setTranslation(glm::vec3(0.0, -0.01, 0.0));
	ground->setRotation(glm::vec3(-90, 0, 0));
	ground->setScale(glm::vec3(100.0, 100.0, 1.0));
    ground->create();
    dynamicsWorld->addRigidBody(ground->rigidBody);

	painter->addMesh(ground, "mesh_ground", "./assets/ground1.jpg", vshader, fshader, 1,glm::mat4(1.0f));
	meshList.push_back(ground);

    
    cube->generateCube();
    cube->setTranslation(glm::vec3(0.0f, 10.0f, 0.0f));
    cube->setRotation(glm::vec3(-90, 0, 0));
    cube->setScale(glm::vec3(4, 4, 4));
    cube->create(1.0f, btVector3(0, 0, 0));
    dynamicsWorld->addRigidBody(cube->rigidBody);
    painter->addMesh(cube, "cube", "./assets/grass.jpg", vshader, fshader, 3, glm::mat4(1.0f));
    meshList.push_back(cube);

    //天空盒
    skybox1->generateSquare(glm::vec3(0.0, 0.0, 0.0));

    skybox1->setTranslation(glm::vec3(0.0, 0.0, -waterScale / 2) + glm::vec3(camera->eye));
    skybox1->setRotation(glm::vec3(0.0, 0.0, 0.0));
    skybox1->setScale(glm::vec3(waterScale*sqrt(2.0), waterScale * sqrt(2.0), waterScale * sqrt(2.0)));

    painter->addMesh(skybox1, "skybox1", "./assets/_skybox_6.jpg", vshader, fshader, 1, glm::mat4(1.0f));
    meshList.push_back(skybox1);

    skybox2->generateSquare(glm::vec3(0.0, 0.0, 0.0));

    skybox2->setTranslation(glm::vec3(0.0, 0.0, waterScale / 2) + glm::vec3(camera->eye));
    skybox2->setRotation(glm::vec3(0.0, 180.0, 0.0));
    skybox2->setScale(glm::vec3(waterScale * sqrt(2.0), waterScale * sqrt(2.0), waterScale * sqrt(2.0)));

    painter->addMesh(skybox2, "skybox2", "./assets/_skybox_8.jpg", vshader, fshader, 1, glm::mat4(1.0f));
    meshList.push_back(skybox2);

    skybox3->generateSquare(glm::vec3(0.0, 0.0, 0.0));

    skybox3->setTranslation(glm::vec3(-waterScale / 2, 0.0, 0.0) + glm::vec3(camera->eye));
    skybox3->setRotation(glm::vec3(0.0, 90.0, 0.0));
    skybox3->setScale(glm::vec3(waterScale * sqrt(2.0), waterScale * sqrt(2.0), waterScale * sqrt(2.0)));

    painter->addMesh(skybox3, "skybox3", "./assets/_skybox_5.jpg", vshader, fshader, 1, glm::mat4(1.0f));
    meshList.push_back(skybox3);

    skybox4->generateSquare(glm::vec3(0.0, 0.0, 0.0));

    skybox4->setTranslation(glm::vec3(waterScale / 2,0.0, 0.0) + glm::vec3(camera->eye));
    skybox4->setRotation(glm::vec3(0.0, -90.0, 0.0));
    skybox4->setScale(glm::vec3(waterScale * sqrt(2.0), waterScale * sqrt(2.0), waterScale * sqrt(2.0)));

    painter->addMesh(skybox4, "skybox4", "./assets/_skybox_7.jpg", vshader, fshader, 1, glm::mat4(1.0f));
    meshList.push_back(skybox4);

    skybox5->generateSquare(glm::vec3(0.0, 0.0, 0.0));

    skybox5->setTranslation(glm::vec3(0.0, waterScale / 2, 0.0) + glm::vec3(camera->eye));
    skybox5->setRotation(glm::vec3(90.0, 0.0, 0.0));
    skybox5->setScale(glm::vec3(waterScale * sqrt(2.0), waterScale * sqrt(2.0), waterScale * sqrt(2.0)));

    painter->addMesh(skybox5, "skybox5", "./assets/_skybox_2.jpg", vshader, fshader, 1,glm::mat4(1.0f));
    meshList.push_back(skybox5);

    skybox6->generateSquare(glm::vec3(0.0, 0.0, 0.0));

    skybox6->setTranslation(glm::vec3(0.0,  - waterScale / 2, 0.0) + glm::vec3(camera->eye));
    skybox6->setRotation(glm::vec3(-90.0, 0.0, 0.0));
    skybox6->setScale(glm::vec3(waterScale * sqrt(2.0), waterScale * sqrt(2.0), waterScale * sqrt(2.0)));

    painter->addMesh(skybox6, "skybox6", "./assets/_skybox_10.jpg", vshader, fshader, 1, glm::mat4(1.0f));
    meshList.push_back(skybox6);
	glClearColor(1.0, 1.0, 1.0, 1.0);
	// glClearColor(0.0, 0.0, 0.0, 1.0);
}
void display()
{
    // #ifdef __APPLE__ // 解决 macOS 10.15 显示画面缩小问题
    // 	glViewport(0, 0, WIDTH * 2, HEIGHT * 2);
    // #endif

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    painter->drawMeshes(light, camera);

    glDepthMask(GL_TRUE);

    //glutSwapBuffers();
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
        default:
            camera->keyboard(key, action, mode);
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
    GLFWwindow* window = glfwCreateWindow(600, 600, "Simple-texture", NULL, NULL);
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
    while (!glfwWindowShouldClose(window))
    {
        dynamicsWorld->stepSimulation(1.f / 60.f, 10);
        cube->update_position();
        painter->replaceMesh(cube, "cube", "./assets/grass.jpg", vshader, fshader, 3, glm::mat4(1.0f));
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
/*
int main() {
    // 初始化物理世界
    btBroadphaseInterface* broadphase = new btDbvtBroadphase();
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
    btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
    glm::vec4 mat = glm::vec4(1, 1, 1, 1);
    // 创建重力
    dynamicsWorld->setGravity(btVector3(0, -9.81, 0));

    // 创建地面刚体
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);
    btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
    btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, groundMotionState, groundShape);
    btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
    dynamicsWorld->addRigidBody(groundRigidBody);

    // 创建一个简单的球形刚体
    btCollisionShape* fallShape = new btSphereShape(1);
    btDefaultMotionState* fallMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 10, 0)));
    btScalar mass = 1.0f;
    btVector3 fallInertia(0, 0, 0);
    fallShape->calculateLocalInertia(mass, fallInertia);
    btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
    btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
    dynamicsWorld->addRigidBody(fallRigidBody);

    // 模拟物理世界
    for (int i = 0; i < 300; i++) {
        dynamicsWorld->stepSimulation(1.f / 60.f, 10);

        btTransform trans;
        fallRigidBody->getMotionState()->getWorldTransform(trans);
        printf("x:%f   y:%f   z:%f\n", trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
    }

    // 清理
    delete dynamicsWorld;
    delete solver;
    delete dispatcher;
    delete collisionConfiguration;
    delete broadphase;

    return 0;
}
*/
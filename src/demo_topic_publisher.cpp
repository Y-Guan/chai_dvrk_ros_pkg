//-------------------------------------------------------------------------------
// Include all the libraries
//-------------------------------------------------------------------------------
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "chai/demo_msg.h"
#include "sensor_msgs/JointState.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf/tf.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <iostream>
#include <ostream>
#include <chai3d.h>
#include <stdlib.h>
#include <stdio.h>
#include "GLFW/glfw3.h"
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>
//#include "../external/chai3d-3.2.0/extras/GLFW/include/GLFW/glfw3.h"
//#include "../glfw-master/include/GLFW/glfw3.h"

#include "../external/chai3d-3.2.0/modules/ODE/src/CODE.h"
//#include "../ODE/src/CODE.h"
//#include "../ODE/src/CODEWorld.h"
//#include "../ODE/src/CODEGenericBody.h"



//using namespace chai3d;
//using namespace std;
////---------------------------------------------------------------------------
//// GENERAL SETTINGS
////---------------------------------------------------------------------------
//
//// stereo Mode
///*
//    C_STEREO_DISABLED:            Stereo is disabled
//    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
//    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
//    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
//*/
//cStereoMode stereoMode = C_STEREO_DISABLED;
//
//// fullscreen mode
//bool fullscreen = false;
//
//// mirrored display
//bool mirroredDisplay = false;
//
//
////---------------------------------------------------------------------------
//// CHAI3D VARIABLES
////---------------------------------------------------------------------------
//
//// a world that contains all objects of the virtual environment
//cWorld* world;
//
//// a camera to render the world in the window display
//cCamera* camera;
//
//// a light source to illuminate the objects in the world
//cSpotLight *light;
//
//// a haptic device handler
//cHapticDeviceHandler* handler;
//
//// a pointer to the current haptic device
//shared_ptr<cGenericHapticDevice> hapticDevice;
//
//// a virtual tool representing the haptic device in the scene
//cGenericTool* tool;
//
//// a label to display the rate [Hz] at which the simulation is running
//cLabel* labelRates;
//
//
////---------------------------------------------------------------------------
//// ODE MODULE VARIABLES
////---------------------------------------------------------------------------
//
//// ODE world
//cODEWorld* ODEWorld;
//
//// ODE objects
//cODEGenericBody* ODEBody0;
//cODEGenericBody* ODEBody1;
//cODEGenericBody* ODEBody2;
//
//// ODE objects
//cODEGenericBody* ODEGPlane0;
//cODEGenericBody* ODEGPlane1;
//cODEGenericBody* ODEGPlane2;
//cODEGenericBody* ODEGPlane3;
//cODEGenericBody* ODEGPlane4;
//cODEGenericBody* ODEGPlane5;
//
//
////---------------------------------------------------------------------------
//// GENERAL VARIABLES
////---------------------------------------------------------------------------
//
//// flag to indicate if the haptic simulation currently running
//bool simulationRunning = false;
//
//// flag to indicate if the haptic simulation has terminated
//bool simulationFinished = true;
//
//// a frequency counter to measure the simulation graphic rate
//cFrequencyCounter freqCounterGraphics;
//
//// a frequency counter to measure the simulation haptic rate
//cFrequencyCounter freqCounterHaptics;
//
//// haptic thread
//cThread* hapticsThread;
//
//// a handle to window display context
//GLFWwindow* window = NULL;
//
//// current width of window
//int width = 0;
//
//// current height of window
//int height = 0;
//
//// swap interval for the display context (vertical synchronization)
//int swapInterval = 1;
//
//
////------------------------------------------------------------------------------
//// DECLARED FUNCTIONS
////------------------------------------------------------------------------------
//
//// callback when the window display is resized
//void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);
//
//// callback when an error GLFW occurs
//void errorCallback(int error, const char* a_description);
//
//// callback when a key is pressed
//void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);
//
//// this function renders the scene
//void updateGraphics(void);
//
//// this function contains the main haptics simulation loop
//void updateHaptics(void);
//
//// this function closes the application
//void close(void);
//
//
//
////===========================================================================
///*
//    DEMO:    01-ODE-cubic.cpp
//
//    This example illustrates the use of the ODE framework for simulating
//    haptic interaction with dynamic bodies. In this scene we create 3
//    cubic meshes that we individually attach to ODE bodies. Haptic interactions
//    are computer by using the finger-proxy haptic model and forces are
//    propagated to the ODE representation.
// */
////===========================================================================
//
//int main(int argc, char* argv[])
//{
//    //-----------------------------------------------------------------------
//    // INITIALIZATION
//    //-----------------------------------------------------------------------
//
//    cout << endl;
//    cout << "-----------------------------------" << endl;
//    cout << "CHAI3D" << endl;
//    cout << "Demo: 01-ODE-cubic" << endl;
//    cout << "Copyright 2003-2016" << endl;
//    cout << "-----------------------------------" << endl << endl << endl;
//    cout << "Keyboard Options:" << endl << endl;
//    cout << "[g] - Enable/Disable gravity" << endl;
//    cout << "[f] - Enable/Disable full screen mode" << endl;
//    cout << "[m] - Enable/Disable vertical mirroring" << endl;
//    cout << "[q] - Exit application" << endl;
//    cout << endl << endl;
//
//
//    //--------------------------------------------------------------------------
//    // OPEN GL - WINDOW DISPLAY
//    //--------------------------------------------------------------------------
//
//    // initialize GLFW library
//    if (!glfwInit())
//    {
//        cout << "failed initialization" << endl;
//        cSleepMs(1000);
//        return 1;
//    }
//
//    // set error callback
//    glfwSetErrorCallback(errorCallback);
//
//    // compute desired size of window
//    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
//    int w = 0.8 * mode->height;
//    int h = 0.5 * mode->height;
//    int x = 0.5 * (mode->width - w);
//    int y = 0.5 * (mode->height - h);
//
//    // set OpenGL version
//    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
//    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
//
//    // set active stereo mode
//    if (stereoMode == C_STEREO_ACTIVE)
//    {
//        glfwWindowHint(GLFW_STEREO, GL_TRUE);
//    }
//    else
//    {
//        glfwWindowHint(GLFW_STEREO, GL_FALSE);
//    }
//
//    // create display context
//    window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
//    if (!window)
//    {
//        cout << "failed to create window" << endl;
//        cSleepMs(1000);
//        glfwTerminate();
//        return 1;
//    }
//
//    // get width and height of window
//    glfwGetWindowSize(window, &width, &height);
//
//    // set position of window
//    glfwSetWindowPos(window, x, y);
//
//    // set key callback
//    glfwSetKeyCallback(window, keyCallback);
//
//    // set resize callback
//    glfwSetWindowSizeCallback(window, windowSizeCallback);
//
//    // set current display context
//    glfwMakeContextCurrent(window);
//
//    // sets the swap interval for the current display context
//    glfwSwapInterval(swapInterval);
//
//    // initialize GLEW library
//#ifdef GLEW_VERSION
//    if (glewInit() != GLEW_OK)
//    {
//        cout << "failed to initialize GLEW library" << endl;
//        glfwTerminate();
//        return 1;
//    }
//#endif
//
//
//    //-----------------------------------------------------------------------
//    // WORLD - CAMERA - LIGHTING
//    //-----------------------------------------------------------------------
//
//    // create a new world.
//    world = new cWorld();
//
//    // set the background color of the environment
//    world->m_backgroundColor.setWhite();
//
//    // create a camera and insert it into the virtual world
//    camera = new cCamera(world);
//    world->addChild(camera);
//
//    // position and orient the camera
//    camera->set(cVector3d (2.5, 0.0, 0.3),    // camera position (eye)
//                cVector3d (0.0, 0.0,-0.5),    // lookat position (target)
//                cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector
//
//    // set the near and far clipping planes of the camera
//    camera->setClippingPlanes(0.01, 10.0);
//
//    // set stereo mode
//    camera->setStereoMode(stereoMode);
//
//    // set stereo eye separation and focal length (applies only if stereo is enabled)
//    camera->setStereoEyeSeparation(0.02);
//    camera->setStereoFocalLength(2.0);
//
//    // set vertical mirrored display mode
//    camera->setMirrorVertical(mirroredDisplay);
//
//    // create a light source
//    light = new cSpotLight(world);
//
//    // attach light to camera
//    world->addChild(light);
//
//    // enable light source
//    light->setEnabled(true);
//
//    // position the light source
//    light->setLocalPos(0.0, 0.0, 1.2);
//
//    // define the direction of the light beam
//    light->setDir(0.0, 0.0,-1.0);
//
//    // set uniform concentration level of light
//    light->setSpotExponent(0.0);
//
//    // enable this light source to generate shadows
//    light->setShadowMapEnabled(true);
//
//    // set the resolution of the shadow map
//    light->m_shadowMap->setQualityLow();
//    //light->m_shadowMap->setQualityMedium();
//
//    // set light cone half angle
//    light->setCutOffAngleDeg(45);
//
//
//    //-----------------------------------------------------------------------
//    // HAPTIC DEVICES / TOOLS
//    //-----------------------------------------------------------------------
//
//    // create a haptic device handler
//    handler = new cHapticDeviceHandler();
//
//    // get access to the first available haptic device found
//    handler->getDevice(hapticDevice, 0);
//
//    // retrieve information about the current haptic device
//    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();
//
//    // create a tool (gripper or pointer)
//    if (hapticDeviceInfo.m_actuatedGripper)
//    {
//        tool = new cToolGripper(world);
//    }
//    else
//    {
//        tool = new cToolCursor(world);
//    }
//
//    // insert tool into world
//    world->addChild(tool);
//
//    // connect the haptic device to the virtual tool
//    tool->setHapticDevice(hapticDevice);
//
//    // map the physical workspace of the haptic device to a larger virtual workspace.
//    tool->setWorkspaceRadius(1.3);
//
//    // define a radius for the virtual tool contact points (sphere)
//    double toolRadius = 0.06;
//    tool->setRadius(toolRadius, toolRadius);
//
//    // enable if objects in the scene are going to rotate of translate
//    // or possibly collide against the tool. If the environment
//    // is entirely static, you can set this parameter to "false"
//    tool->enableDynamicObjects(true);
//
//    // haptic forces are enabled only if small forces are first sent to the device;
//    // this mode avoids the force spike that occurs when the application starts when
//    // the tool is located inside an object for instance.
//    tool->setWaitForSmallForce(true);
//
//    // start the haptic tool
//    tool->start();
//
//
//    //--------------------------------------------------------------------------
//    // WIDGETS
//    //--------------------------------------------------------------------------
//
//    // create a font
//    cFontPtr font = NEW_CFONTCALIBRI20();
//
//    // create a label to display the haptic and graphic rate of the simulation
//    labelRates = new cLabel(font);
//    labelRates->m_fontColor.setBlack();
//    camera->m_frontLayer->addChild(labelRates);
//
//
//    //-----------------------------------------------------------------------
//    // CREATE ODE WORLD AND OBJECTS
//    //-----------------------------------------------------------------------
//
//    //////////////////////////////////////////////////////////////////////////
//    // ODE WORLD
//    //////////////////////////////////////////////////////////////////////////
//
//    // read the scale factor between the physical workspace of the haptic
//    // device and the virtual workspace defined for the tool
//    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();
//
//    // stiffness properties
//    double maxStiffness	= hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;
//
//    // create an ODE world to simulate dynamic bodies
//    ODEWorld = new cODEWorld(world);
//
//    // add ODE world as a node inside world
//    world->addChild(ODEWorld);
//
//    // set some gravity
//    ODEWorld->setGravity(cVector3d(0.00, 0.00,-9.81));
//
//    // define damping properties
//    ODEWorld->setAngularDamping(0.00002);
//    ODEWorld->setLinearDamping(0.00002);
//
//
//    //////////////////////////////////////////////////////////////////////////
//    // 3 ODE BLOCKS
//    //////////////////////////////////////////////////////////////////////////
//
//    // create a new ODE object that is automatically added to the ODE world
//    ODEBody0 = new cODEGenericBody(ODEWorld);
//    ODEBody1 = new cODEGenericBody(ODEWorld);
//    ODEBody2 = new cODEGenericBody(ODEWorld);
//
//    // create a virtual mesh  that will be used for the geometry representation of the dynamic body
//    cMesh* object0 = new cMesh();
//    cMesh* object1 = new cMesh();
//    cMesh* object2 = new cMesh();
//
//    // create a cube mesh
//    double size = 0.40;
//    cCreateBox(object0, size, size, size);
//    object0->createAABBCollisionDetector(toolRadius);
//
//    cCreateBox(object1, size, size, size);
//    object1->createAABBCollisionDetector(toolRadius);
//
//    cCreateBox(object2, size, size, size);
//    object2->createAABBCollisionDetector(toolRadius);
//
//    // define some material properties for each cube
//    cMaterial mat0, mat1, mat2;
//    mat0.setRedIndian();
//    mat0.setStiffness(0.3 * maxStiffness);
//    mat0.setDynamicFriction(0.6);
//    mat0.setStaticFriction(0.6);
//    object0->setMaterial(mat0);
//
//    mat1.setBlueRoyal();
//    mat1.setStiffness(0.3 * maxStiffness);
//    mat1.setDynamicFriction(0.6);
//    mat1.setStaticFriction(0.6);
//    object1->setMaterial(mat1);
//
//    mat2.setGreenDarkSea();
//    mat2.setStiffness(0.3 * maxStiffness);
//    mat2.setDynamicFriction(0.6);
//    mat2.setStaticFriction(0.6);
//    object2->setMaterial(mat2);
//
//    // add mesh to ODE object
//    ODEBody0->setImageModel(object0);
//    ODEBody1->setImageModel(object1);
//    ODEBody2->setImageModel(object2);
//
//    // create a dynamic model of the ODE object. Here we decide to use a box just like
//    // the object mesh we just defined
//    ODEBody0->createDynamicBox(size, size, size);
//    ODEBody1->createDynamicBox(size, size, size);
//    ODEBody2->createDynamicBox(size, size, size);
//
//    // define some mass properties for each cube
//    ODEBody0->setMass(0.05);
//    ODEBody1->setMass(0.05);
//    ODEBody2->setMass(0.05);
//
//    // set position of each cube
//    ODEBody0->setLocalPos(0.0,-0.6,-0.5);
//    ODEBody1->setLocalPos(0.0, 0.6,-0.5);
//    ODEBody2->setLocalPos(0.0, 0.0,-0.5);
//
//    // rotate central cube 45 degrees around z-axis
//    ODEBody0->rotateAboutGlobalAxisDeg(0,0,1, 45);
//
//
//    //////////////////////////////////////////////////////////////////////////
//    // 6 ODE INVISIBLE WALLS
//    //////////////////////////////////////////////////////////////////////////
//
//    // we create 6 static walls to contains the 3 cubes within a limited workspace
//    ODEGPlane0 = new cODEGenericBody(ODEWorld);
//    ODEGPlane1 = new cODEGenericBody(ODEWorld);
//    ODEGPlane2 = new cODEGenericBody(ODEWorld);
//    ODEGPlane3 = new cODEGenericBody(ODEWorld);
//    ODEGPlane4 = new cODEGenericBody(ODEWorld);
//    ODEGPlane5 = new cODEGenericBody(ODEWorld);
//
//    w = 1.0;
//    ODEGPlane0->createStaticPlane(cVector3d(0.0, 0.0,  2.0 * w), cVector3d(0.0, 0.0 ,-1.0));
//    ODEGPlane1->createStaticPlane(cVector3d(0.0, 0.0, -w), cVector3d(0.0, 0.0 , 1.0));
//    ODEGPlane2->createStaticPlane(cVector3d(0.0,  w, 0.0), cVector3d(0.0,-1.0, 0.0));
//    ODEGPlane3->createStaticPlane(cVector3d(0.0, -w, 0.0), cVector3d(0.0, 1.0, 0.0));
//    ODEGPlane4->createStaticPlane(cVector3d( w, 0.0, 0.0), cVector3d(-1.0,0.0, 0.0));
//    ODEGPlane5->createStaticPlane(cVector3d(-0.8 * w, 0.0, 0.0), cVector3d( 1.0,0.0, 0.0));
//
//
//    //////////////////////////////////////////////////////////////////////////
//    // GROUND
//    //////////////////////////////////////////////////////////////////////////
//
//    // create a mesh that represents the ground
//    cMesh* ground = new cMesh();
//    ODEWorld->addChild(ground);
//
//    // create a plane
//    double groundSize = 3.0;
//    cCreatePlane(ground, groundSize, groundSize);
//
//    // position ground in world where the invisible ODE plane is located (ODEGPlane1)
//    ground->setLocalPos(0.0, 0.0, -1.0);
//
//    // define some material properties and apply to mesh
//    cMaterial matGround;
//    matGround.setStiffness(0.3 * maxStiffness);
//    matGround.setDynamicFriction(0.2);
//    matGround.setStaticFriction(0.0);
//    matGround.setWhite();
//    matGround.m_emission.setGrayLevel(0.3);
//    ground->setMaterial(matGround);
//
//    // setup collision detector
//    ground->createAABBCollisionDetector(toolRadius);
//
//
//    //-----------------------------------------------------------------------
//    // START SIMULATION
//    //-----------------------------------------------------------------------
//
//    // create a thread which starts the main haptics rendering loop
//    hapticsThread = new cThread();
//    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);
//
//    // setup callback when application exits
//    atexit(close);
//
//
//    //--------------------------------------------------------------------------
//    // MAIN GRAPHIC LOOP
//    //--------------------------------------------------------------------------
//
//    // call window size callback at initialization
//    windowSizeCallback(window, width, height);
//
//    // main graphic loop
//    while (!glfwWindowShouldClose(window))
//    {
//        // get width and height of window
//        glfwGetWindowSize(window, &width, &height);
//
//        // render graphics
//        updateGraphics();
//
//        // swap buffers
//        glfwSwapBuffers(window);
//
//        // process events
//        glfwPollEvents();
//
//        // signal frequency counter
//        freqCounterGraphics.signal(1);
//    }
//
//    // close window
//    glfwDestroyWindow(window);
//
//    // terminate GLFW library
//    glfwTerminate();
//
//    // exit
//    return 0;
//}
//
////---------------------------------------------------------------------------
//
//void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
//{
//    // update window size
//    width  = a_width;
//    height = a_height;
//}
//
////------------------------------------------------------------------------------
//
//void errorCallback(int a_error, const char* a_description)
//{
//    cout << "Error: " << a_description << endl;
//}
//
////---------------------------------------------------------------------------
//
//void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
//{
//    // filter calls that only include a key press
//    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
//    {
//        return;
//    }
//
//        // option - exit
//    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
//    {
//        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
//    }
//
//        // option - enable/disable gravity
//    else if (a_key == GLFW_KEY_G)
//    {
//        if (ODEWorld->getGravity().length() > 0.0)
//        {
//            ODEWorld->setGravity(cVector3d(0.0, 0.0, 0.0));
//        }
//        else
//        {
//            ODEWorld->setGravity(cVector3d(0.0, 0.0,-9.81));
//        }
//    }
//
//        // option - toggle fullscreen
//    else if (a_key == GLFW_KEY_F)
//    {
//        // toggle state variable
//        fullscreen = !fullscreen;
//
//        // get handle to monitor
//        GLFWmonitor* monitor = glfwGetPrimaryMonitor();
//
//        // get information about monitor
//        const GLFWvidmode* mode = glfwGetVideoMode(monitor);
//
//        // set fullscreen or window mode
//        if (fullscreen)
//        {
//            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
//            glfwSwapInterval(swapInterval);
//        }
//        else
//        {
//            int w = 0.8 * mode->height;
//            int h = 0.5 * mode->height;
//            int x = 0.5 * (mode->width - w);
//            int y = 0.5 * (mode->height - h);
//            glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
//            glfwSwapInterval(swapInterval);
//        }
//    }
//
//        // option - toggle vertical mirroring
//    else if (a_key == GLFW_KEY_M)
//    {
//        mirroredDisplay = !mirroredDisplay;
//        camera->setMirrorVertical(mirroredDisplay);
//    }
//
//
//
//
//        // tool forward
//    else if (a_key == GLFW_KEY_UP)
//    {
//        cVector3d tool_pos = tool->getDeviceGlobalPos();
//        tool_pos(0) = tool_pos(0) - 0.01;
//        tool->setDeviceGlobalPos(tool_pos);
//
//    }
//        // tool back
//    else if (a_key == GLFW_KEY_DOWN)
//    {
//        cVector3d tool_pos = tool->getDeviceGlobalPos();
//        tool_pos(0) = tool_pos(0) + 0.01;
//        tool->setDeviceGlobalPos(tool_pos);
//
//    }
//        // tool left
//    else if (a_key == GLFW_KEY_LEFT)
//    {
//        cVector3d tool_pos = tool->getDeviceGlobalPos();
//        tool_pos(1) = tool_pos(1) - 0.01;
//        tool->setDeviceGlobalPos(tool_pos);
//
//    }
//        // tool right
//    else if (a_key == GLFW_KEY_RIGHT)
//    {
//        cVector3d tool_pos = tool->getDeviceGlobalPos();
//        tool_pos(1) = tool_pos(1) + 0.01;
//        tool->setDeviceGlobalPos(tool_pos);
//
//    }
//        // tool up
//    else if (a_key == GLFW_KEY_P)
//    {
//        cVector3d tool_pos = tool->getDeviceGlobalPos();
//        tool_pos(2) = tool_pos(2) + 0.01;
//        tool->setDeviceGlobalPos(tool_pos);
//
//    }
//        // tool down
//    else if (a_key == GLFW_KEY_L)
//    {
//        cVector3d tool_pos = tool->getDeviceGlobalPos();
//        tool_pos(2) = tool_pos(2) - 0.01;
//        tool->setDeviceGlobalPos(tool_pos);
//
//    }
//
//
//
//
//
//
//
//}
//
////---------------------------------------------------------------------------
//
//void close(void)
//{
//    // stop the simulation
//    simulationRunning = false;
//
//    // wait for graphics and haptics loops to terminate
//    while (!simulationFinished) { cSleepMs(100); }
//
//    // close haptic device
//    tool->stop();
//
//    // delete resources
//    delete hapticsThread;
//    delete world;
//    delete handler;
//}
//
////---------------------------------------------------------------------------
//
//void updateGraphics(void)
//{
//    /////////////////////////////////////////////////////////////////////
//    // UPDATE WIDGETS
//    /////////////////////////////////////////////////////////////////////
//
//    // update haptic and graphic rate data
//    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
//                        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");
//
//    // update position of label
//    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);
//
//
//    /////////////////////////////////////////////////////////////////////
//    // RENDER SCENE
//    /////////////////////////////////////////////////////////////////////
//
//    // update shadow maps (if any)
//    world->updateShadowMaps(false, mirroredDisplay);
//
//    // render world
//    camera->renderView(width, height);
//
//    // wait until all GL commands are completed
//    glFinish();
//
//    // check for any OpenGL errors
//    GLenum err = glGetError();
//    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));
//}
//
////---------------------------------------------------------------------------
//
//void updateHaptics(void)
//{
//    // simulation in now running
//    simulationRunning  = true;
//    simulationFinished = false;
//
//    // reset clock
//    cPrecisionClock clock;
//    clock.reset();
//
//    // main haptic simulation loop
//    while(simulationRunning)
//    {
//        /////////////////////////////////////////////////////////////////////
//        // SIMULATION TIME
//        /////////////////////////////////////////////////////////////////////
//
//        // stop the simulation clock
//        clock.stop();
//
//        // read the time increment in seconds
//        double timeInterval = cClamp(clock.getCurrentTimeSeconds(), 0.0001, 0.001);
//
//        // restart the simulation clock
//        clock.reset();
//        clock.start();
//
//        // signal frequency counter
//        freqCounterHaptics.signal(1);
//
//
//        /////////////////////////////////////////////////////////////////////
//        // HAPTIC FORCE COMPUTATION
//        /////////////////////////////////////////////////////////////////////
//
//        // compute global reference frames for each object
//        world->computeGlobalPositions(true);
//
//        // update position and orientation of tool
//        tool->updateFromDevice();
//
//        // compute interaction forces
//        tool->computeInteractionForces();
//
//        // send forces to haptic device
//        tool->applyToDevice();
//
//
//        /////////////////////////////////////////////////////////////////////
//        // DYNAMIC SIMULATION
//        /////////////////////////////////////////////////////////////////////
//
//        // for each interaction point of the tool we look for any contact events
//        // with the environment and apply forces accordingly
//        int numInteractionPoints = tool->getNumHapticPoints();
//        for (int i=0; i<numInteractionPoints; i++)
//        {
//            // get pointer to next interaction point of tool
//            cHapticPoint* interactionPoint = tool->getHapticPoint(i);
//
//            // check all contact points
//            int numContacts = interactionPoint->getNumCollisionEvents();
//            for (int i=0; i<numContacts; i++)
//            {
//                cCollisionEvent* collisionEvent = interactionPoint->getCollisionEvent(i);
//
//                // given the mesh object we may be touching, we search for its owner which
//                // could be the mesh itself or a multi-mesh object. Once the owner found, we
//                // look for the parent that will point to the ODE object itself.
//                cGenericObject* object = collisionEvent->m_object->getOwner()->getOwner();
//
//                // cast to ODE object
//                cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(object);
//
//                // if ODE object, we apply interaction forces
//                if (ODEobject != NULL)
//                {
//                    ODEobject->addExternalForceAtPoint(-0.3 * interactionPoint->getLastComputedForce(),
//                                                       collisionEvent->m_globalPos);
//                }
//            }
//        }
//
//        // update simulation
//        ODEWorld->updateDynamics(timeInterval);
//    }
//
//    // exit haptics thread
//    simulationFinished = true;
//}
//







using namespace chai3d;
using namespace std;

//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
//cWorld* world;
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// You may need to define the objects here.
// ...
// a few spherical objects
cShapeSphere* object0;
cShapeLine* force_line;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;
cToolGripper* gripper_tool;

// a virtual object
cMultiMesh* shaft;
cMultiMesh* caudier_L;
cMultiMesh* caudier_R;

cMesh* mesh_arrow_x;
cMesh* mesh_arrow_y;
cMesh* mesh_arrow_z;
cMesh* mesh_obj;

// a colored background
cBackground* background;

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// a flag that indicates if the haptic simulation is currently running
bool simulationRunning = false;

// a flag that indicates if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a first window
GLFWwindow* window1 = NULL;
int width1 = 0;
int height1 = 0;

// a second window
GLFWwindow* window2 = NULL;
int width2 = 0;
int height2 = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

// root resource path
string resourceRoot;

//------------------------------------------------------------------------------
// Declared local variables
//------------------------------------------------------------------------------
double gripper_angle = 0.0;
double gripper_angle_lim_max = 45.0;

//------------------------------------------------------------------------------
// DECLARED MACROS
//------------------------------------------------------------------------------

// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())

// toggle when debugging in CLion / Or just ignore this macro
#define DEBUG_MODE

//#define C_USE_OPENGL

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback for tf listener
void tf_position_cb(const geometry_msgs::Vector3::ConstPtr & msg);

void pose_subscriber_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

// callback when the window1 display is resized
void windowSizeCallback1(GLFWwindow* a_window, int a_width, int a_height);
// callback when the window2 display is resized
void windowSizeCallback2(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene1
void updateGraphics1(void);
// this function renders the scene2
void updateGraphics2(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);


//--------------------------------------------------------------------------
// MAIN
//--------------------------------------------------------------------------

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // Get all the arguments
    //--------------------------------------------------------------------------
    printf("argc = %d\n", argc);
    for (int i = 0; i < argc; ++i) {
        printf("argv[%d] = %s\n", i, argv[0]);
    }

    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D ROS APPLICATION for dVRK MTM" << endl;
    cout << "Yuan Guan's Template" << endl;
    cout << "University College London" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;

    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);
    cout << "--------------------------------------------------------------------------------------" << endl;
    std::cout << "Current resource root is : " << resourceRoot  << endl;
    cout << "--------------------------------------------------------------------------------------" << endl << endl;


    //--------------------------------------------------------------------------
    // Init ROS Node
    //--------------------------------------------------------------------------

    // init a node with name "demo_topic_publisher_node"
    ros::init(argc, argv, "chai3d_node");
    // create a object for the node to communicate with ROS system
    ros::NodeHandle node_obj;

    /* ######################################################
    Create a topic publisher:
     - topic name: /dvrk/MTMR/state_joint_current
     - message type: std::msgs::Int32
     - queue size: 10 (set to high if sending rate is high)
    #######################################################*/
    ros::Publisher jointState_publisher = node_obj.advertise<sensor_msgs::JointState>("/dvrk/MTMR/state_joint_current",10);

    // set the frequency of sending data
    ros::Rate loop_rate(60);

    /* ######################################################
    Create a topic publisher:
     - topic name: /dvrk/MTMR/position_cartesian_current
     - message type: geometry_msgs::PoseStampedConstPtr
     - queue size: 10 (set to high if sending rate is high)
    #######################################################*/
    ros::Subscriber pose_subscriber = node_obj.subscribe<geometry_msgs::PoseStamped>("/dvrk/MTMR/position_cartesian_current", 10, pose_subscriber_cb);

    //--------------------------------------------------------------------------
    // Init ROS msgs
    //--------------------------------------------------------------------------

    ///////////////////////////////////////////
    // MTM joint state msg
    ///////////////////////////////////////////
    sensor_msgs::JointState msg;
    msg.position.resize(8);
    msg.name.resize(8);
    msg.name[0] = "outer_yaw";
    msg.name[1] = "shoulder_pitch";
    msg.name[2] = "shoulder_pitch_parallel";
    msg.name[3] = "elbow_pitch";
    msg.name[4] = "wrist_platform";
    msg.name[5] = "wrist_pitch";
    msg.name[6] = "wrist_yaw";
    msg.name[7] = "wrist_roll";
    msg.position[0] = 0.0;
    msg.position[1] = 0.0;
    msg.position[2] = 0.0;
    msg.position[3] = 0.0;
    msg.position[4] = 0.0;
    msg.position[5] = 0.0;
    msg.position[6] = 0.0;
    msg.position[7] = 0.0;
    double increment0 = 0.01;
    double increment1 = 0.01;
    double increment2 = 0.01;
    double increment3 = 0.01;

    ///////////////////////////////////////////
    // Init tf listener
    ///////////////////////////////////////////
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ///////////////////////////////////////////
    // Init msg buffers
    ///////////////////////////////////////////
    //geometry_msgs::PoseStamped cur_pose, pre_pose;

    //--------------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        std::cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    glfwSetErrorCallback(errorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int space = 10;
    int w = 0.45 * mode->width;
    int h = 0.6 * mode->height;
    int x1 = 0.5 * mode->width - w - space;
    int y1 = 0.5 * (mode->height - h);
    int x2 = 0.5 * mode->width + space;
    int y2 = 0.5 * (mode->height - h);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    ////////////////////////////////////////////////////////////////////////////
    // SETUP WINDOW 1
    ////////////////////////////////////////////////////////////////////////////

    // create display context
    window1 = glfwCreateWindow(w, h, "Left eye", NULL, NULL);
    if (!window1)
    {
        std::cout << "failed to create window1" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window1, &width1, &height1);

    // set position of window
    glfwSetWindowPos(window1, x1, y1);

    // set key callback
    glfwSetKeyCallback(window1, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window1, windowSizeCallback1);

    // set current display context
    glfwMakeContextCurrent(window1);

    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);


    ////////////////////////////////////////////////////////////////////////////
    // SETUP WINDOW 2
    ////////////////////////////////////////////////////////////////////////////

    // create display context and share GPU data with window 0
    window2 = glfwCreateWindow(w, h, "Right eye", NULL, window1);
    if (!window2)
    {
        std::cout << "failed to create window2" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window2, &width2, &height2);

    // set position of window
    glfwSetWindowPos(window2, x2, y2);

    // set key callback
    glfwSetKeyCallback(window2, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window2, windowSizeCallback2);

    // set current display context
    glfwMakeContextCurrent(window2);

    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);

    ////////////////////////////////////////////////////////////////////////////
    // GLEW
    ////////////////////////////////////////////////////////////////////////////

    // initialize GLEW library
#ifdef GLEW_VERSION
    if (glewInit() != GLEW_OK)
    {
        std::cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif


    //--------------------------------------------------------------------------
    // WORLD
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setWhite();


    //--------------------------------------------------------------------------
    // CAMERA
    //--------------------------------------------------------------------------

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    cVector3d camera_look_at_position(0.25, 0.5, 0.7);
//    cVector3d camera_position(-0.0, -5.0, 0.0);
    cVector3d camera_position(-0.0, -1.5, 1.0);
    cVector3d upward_direction(0.0, 0.0, 1.0);
    camera->set(camera_position,             // camera position (eye)
                camera_look_at_position,    // look-at position (target)
                upward_direction);         // direction of the (up) vector

    // set the near and far clipping planes of the camera
    // anything in front or behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10);

    // set stereo mode
    camera->setStereoMode(C_STEREO_PASSIVE_DUAL_DISPLAY);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.065);
    camera->setStereoFocalLength(3.0);

    //--------------------------------------------------------------------------
    // LIGHTING
    //--------------------------------------------------------------------------

    // create a light source
    light = new cDirectionalLight(world);

    // add light to world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // define the direction of the light beam
    light->setDir(-1.0,-1.0, -1.0);

    // set lighting conditions
    light->m_ambient.set(0.4f, 0.4f, 0.4f);
    light->m_diffuse.set(0.8f, 0.8f, 0.8f);
    light->m_specular.set(1.0f, 1.0f, 1.0f);


    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    ////////////////////////////////////////////////////////
    // Cursor tool
    ////////////////////////////////////////////////////////
    // create a tool (cursor) and insert into the world
    tool = new cToolCursor(world);
    world->addChild(tool);

    // connect the haptic device to the virtual tool
    //tool->setHapticDevice(hapticDevice);

    // define the radius of the tool (sphere)
    double toolRadius = 0.02;

    // define a radius for the tool
    tool->setRadius(toolRadius);

    // hide the device sphere. only show proxy.
    tool->setShowContactPoints(false, false);

    // create a black cursor
    tool->m_hapticPoint->m_sphereProxy->m_material->setBlack();

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.0);

    // haptic forces are enabled only if small forces are first sent to the device;
    // this mode avoids the force spike that occurs when the application starts when
    // the tool is located inside an object for instance.
    tool->setWaitForSmallForce(true);

    // start the haptic tool
    tool->start();

    ////////////////////////////////////////////////////////
    // gripper tool
    ////////////////////////////////////////////////////////
    // create a tool (cursor) and insert into the world
    gripper_tool = new cToolGripper(world);
    world->addChild(gripper_tool);

    // define a radius for the tool
    double gripper_toolRadius = 0.01;
    gripper_tool->setRadius(gripper_toolRadius);

    gripper_tool->setHapticDevice(hapticDevice);

    // if the device has a gripper, enable the gripper to simulate a user switch
    hapticDevice->setEnableGripperUserSwitch(true);



    force_line = new cShapeLine();
    world->addChild(force_line);



    // hide the device sphere. only show proxy.
//    gripper_tool->setShowContactPoints(true,true);

    // create black proxy points
    gripper_tool->m_hapticPointThumb->m_sphereProxy->m_material->setBlack();
    gripper_tool->m_hapticPointFinger->m_sphereProxy->m_material->setWhite();
//    gripper_tool->m_hapticPointThumb->m_sphereProxy->setLocalPos(cVector3d(0.5, 0.0, 0.0));

    cout << "Thumb point global position is : " << gripper_tool->m_hapticPointThumb->m_sphereProxy->getGlobalPos() << endl;
    cout << "Finger point global position is : " << gripper_tool->m_hapticPointFinger->m_sphereProxy->getGlobalPos() << endl;
    cout << "Thumb point local position is : " << gripper_tool->m_hapticPointThumb->m_sphereProxy->getLocalPos() << endl;
    cout << "Finger point local position is : " << gripper_tool->m_hapticPointFinger->m_sphereProxy->getLocalPos() << endl;

    // map the physical workspace of the haptic device to a larger virtual workspace.
    gripper_tool->setWorkspaceRadius(1.0);

    // haptic forces are enabled only if small forces are first sent to the device;
    // this mode avoids the force spike that occurs when the application starts when
    // the tool is located inside an object for instance.
    gripper_tool->setWaitForSmallForce(true);

    // start the haptic tool
    gripper_tool->start();

    //--------------------------------------------------------------------------
    // CREATE OBJECTS
    //--------------------------------------------------------------------------

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // properties
    double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

    bool fileload;

    /////////////////////////////////////////////////////////////////////////
    // OBJECTs for gripper
    /////////////////////////////////////////////////////////////////////////
    //
    ///////////////////////////////////////////////////////////////
    // shaft
    ///////////////////////////////////////////////////////////////
    // create a virtual mesh
    shaft = new cMultiMesh();

    // attach scope to tool
    gripper_tool->m_image = shaft;

    // load an object file
//    fileload = scope->loadFromFile(RESOURCE_PATH("../../../src/chai/external/chai3d-3.2.0/bin/resources/models/endoscope/endoscope.3ds"));
    fileload = shaft->loadFromFile(RESOURCE_PATH("../../../src/chai/external/chai3d-3.2.0/bin/resources/models/endoscope/tool_wrist_caudier_link_1.stl"));

    if (!fileload)
    {
#if defined(_MSVC)
        fileload = scope->loadFromFile("../../../bin/resources/models/endoscope/endoscope.3ds");
#endif
#if defined(DEBUG_MODE)
//        fileload = scope->loadFromFile(RESOURCE_PATH("../../../../external/chai3d-3.2.0/bin/resources/models/endoscope/endoscope.3ds"));
        fileload = shaft->loadFromFile(RESOURCE_PATH("../../../../external/chai3d-3.2.0/bin/resources/models/endoscope/tool_wrist_caudier_link_1_shaft.stl"));
#endif
    }
    if (!fileload)
    {
        cout << "Error - 3D Model failed to load correctly." << endl;
        close();
        return (-1);
    }

    // disable culling so that faces are rendered on both sides
    shaft->setUseCulling(false);

    // scale model
    shaft->scale(5);

    // use display list for faster rendering
    shaft->setUseDisplayList(true);

    // position object in scene
//    shaft->rotateExtrinsicEulerAnglesDeg(0.0, 0.0, 0.0, C_EULER_ORDER_XYZ);
    shaft->rotateExtrinsicEulerAnglesDeg(0.0, -90.0, 0.0, C_EULER_ORDER_XYZ);

    cout << "shaft transparency? : " << shaft->getUseTransparency() << endl;

    ///////////////////////////////////////////////////////////////
    // left caudier
    ///////////////////////////////////////////////////////////////
    // create a virtual mesh
    caudier_L = new cMultiMesh();

    shaft->addChild(caudier_L);

    // load an object file
    fileload = caudier_L->loadFromFile(RESOURCE_PATH("../../../src/chai/external/chai3d-3.2.0/bin/resources/models/endoscope/tool_wrist_caudier_link_2.stl"));

    if (!fileload)
    {
#if defined(DEBUG_MODE)
        fileload = caudier_L->loadFromFile(RESOURCE_PATH("../../../../external/chai3d-3.2.0/bin/resources/models/endoscope/tool_wrist_caudier_link_2.stl"));
#endif
    }
    if (!fileload)
    {
        cout << "Error - 3D Model failed to load correctly." << endl;
        close();
        return (-1);
    }

    // disable culling so that faces are rendered on both sides
    caudier_L->setUseCulling(false);

    // scale model
    caudier_L->scale(5);

    // use display list for faster rendering
    caudier_L->setUseDisplayList(true);

    // position object in scene (translate and rotate in shaft frame)
    caudier_L->rotateExtrinsicEulerAnglesDeg(0.0, 90.0, 0.0, C_EULER_ORDER_XYZ);
    caudier_L->translate(cVector3d(0.039/4, 0.0, 0.0));

    cout << "left caudier transparency? : " << caudier_L->getUseTransparency() << endl;

    ///////////////////////////////////////////////////////////////
    // Right caudier
    ///////////////////////////////////////////////////////////////
    // create a virtual mesh
    caudier_R = new cMultiMesh();

    shaft->addChild(caudier_R);

    // load an object file
    fileload = caudier_R->loadFromFile(RESOURCE_PATH("../../../src/chai/external/chai3d-3.2.0/bin/resources/models/endoscope/tool_wrist_caudier_link_2.stl"));

    if (!fileload)
    {
#if defined(DEBUG_MODE)
        fileload = caudier_R->loadFromFile(RESOURCE_PATH("../../../../external/chai3d-3.2.0/bin/resources/models/endoscope/tool_wrist_caudier_link_2.stl"));
#endif
    }
    if (!fileload)
    {
        cout << "Error - 3D Model failed to load correctly." << endl;
        close();
        return (-1);
    }

    // disable culling so that faces are rendered on both sides
    caudier_R->setUseCulling(false);

    // scale model
    caudier_R->scale(5);

    // use display list for faster rendering
    caudier_R->setUseDisplayList(true);

    // position object in scene
    caudier_R->rotateExtrinsicEulerAnglesDeg(0.0, -90, 0.0, C_EULER_ORDER_XYZ);
    caudier_R->translate(cVector3d((0.039-0.078)/4, 0.0, -0.0));

    cout << "right caudier transparency? : " << caudier_R->getUseTransparency() << endl;

//    cVector3d the_axis;
//    double the_angle;
//    cout << "local rotation is : " << caudier_R->getLocalRot().toAxisAngle() << endl;


    /////////////////////////////////////////////////////////////////////////
    // OTHER OBJECTS
    /////////////////////////////////////////////////////////////////////////
    // ...
    /////////////////////////////////////////////////////////////////////////
    // OBJECT 0: "MAGNET"
    /////////////////////////////////////////////////////////////////////////

    // create a sphere and define its radius
    object0 = new cShapeSphere(0.3);

    // add object to world
    world->addChild(object0);

    // set the position of the object at the center of the world
    object0->setLocalPos(1.0, 0.0, 0.0);

    // load texture map
    object0->m_texture = cTexture2d::create();
    fileload = object0->m_texture->loadFromFile(RESOURCE_PATH("../../../src/chai/external/chai3d-3.2.0/bin/resources/images/spheremap-3.jpg"));
    if (fileload) {
        cout << "Texture map image Found" << endl;
    }
    else {
        std::cout << "Error0 - Texture image failed to load correctly." << endl;
#if defined(_MSVC)
        fileload = object0->m_texture->loadFromFile("../../../src/chai/external/chai3d-3.2.0/bin/resources/images/spheremap-3.jpg");
#endif
#if defined(DEBUG_MODE)
        fileload = object0->m_texture->loadFromFile(RESOURCE_PATH("../../../../external/chai3d-3.2.0/bin/resources/images/spheremap-5.jpg"));
#endif
        if (fileload) {
            cout << "Texture map image Found" << endl;
        }
    }
    if (!fileload)
    {
        std::cout << "Error1 - Texture image failed to load correctly." << endl;
        close();
        return (-1);
    }

    // set graphic properties
    object0->m_texture->setSphericalMappingEnabled(true);
    object0->setUseTexture(true);
    object0->m_material->setWhite();

    // get properties of haptic device
    double maxLinearForce = cMin(hapticDeviceInfo.m_maxLinearForce, 7.0);
    double maxDamping   = hapticDeviceInfo.m_maxLinearDamping / workspaceScaleFactor;

//    // set haptic properties
//    object0->m_material->setStiffness(0.4 * maxStiffness);          // % of maximum linear stiffness
//    object0->m_material->setMagnetMaxForce(0.6 * maxLinearForce);   // % of maximum linear force
//    object0->m_material->setMagnetMaxDistance(0.15);
//    object0->m_material->setViscosity(0.1 * maxDamping);            // % of maximum linear damping
//    // create a haptic surface effect
//    object0->createEffectSurface();
//    // create a haptic magnetic effect
//    object0->createEffectMagnetic();
//    // create a haptic viscous effect
//    object0->createEffectViscosity();

    // fluid
//    object0->m_material->m_ambient.set(0.1, 0.1, 0.6, 0.5);
//    object0->m_material->m_diffuse.set(0.3, 0.3, 0.9, 0.5);
//    object0->m_material->m_specular.set(1.0, 1.0, 1.0, 0.5);
//    // set haptic properties
//    object0->m_material->setViscosity(0.9 * maxDamping);    // % of maximum linear damping
//    // create a haptic viscous effect
//    object0->createEffectViscosity();

    // stick slip
//    // set haptic properties
//    object0->m_material->setStickSlipForceMax(0.3 * maxLinearForce);// % of maximum linear force
//    object0->m_material->setStickSlipStiffness(0.7 * maxStiffness); // % of maximum linear stiffness
//    // create a haptic stick-slip effect
//    object0->createEffectStickSlip();

    // vibration
//    // set haptic properties
//    object0->m_material->setVibrationFrequency(50);
//    object0->m_material->setVibrationAmplitude(0.1 * maxLinearForce);   // % of maximum linear force
//    object0->m_material->setStiffness(0.1 * maxStiffness);              // % of maximum linear stiffness
//    // create a haptic vibration effect
//    object0->createEffectVibration();
//    // create a haptic surface effect
//    object0->createEffectSurface();

    // normal surface effect
    object0->m_material->setStiffness(0.4 * maxStiffness);          // % of maximum linear stiffness
    object0->createEffectSurface();


    object0->setShowCollisionDetector(true,true);


//    object0->setUseTransparency(true, false);

    object0->setTransparencyLevel(0.5,true,true,true);

    cout << "object using transparency is : " << object0->getUseTransparency() << endl;

//    object0->setCollisionDetector(cGenericCollision )

    cout << "Object Collision? : " << object0->getShowCollisionDetector() << endl;



    //////////////////////////////////////////////////////////////
    // mesh for arrow
    //////////////////////////////////////////////////////////////
    // create a virtual mesh
    mesh_arrow_z = new cMesh();

    // add object to world
    world->addChild(mesh_arrow_z);

    // build mesh using a cylinder primitive
    cCreateArrow(mesh_arrow_z,       // mesh
                 1.0,              // length
                 0.01,             // radiusShaft
                 0.1,             // lengthTip
                 0.03,              // radiusTip
                 false,            // tipsAtBothExtremities?
                 32,               // numSides
                 cVector3d(0,0,1),  // direction
                 cVector3d(0,0,0)  // position
                 //cColorf(R,G,B,alpha) // colour
    );


    // set material color
    mesh_arrow_z->m_material->setBlue();

    // set haptic properties
    mesh_arrow_z->m_material->setStiffness(0.5 * maxStiffness);

    // build collision detection tree
    mesh_arrow_z->createAABBCollisionDetector(gripper_toolRadius);


    ////////////////////////////////////////////////////////////////////////
    // create a virtual mesh
    mesh_arrow_x = new cMesh();

    // add object to world
    world->addChild(mesh_arrow_x);

    // build mesh using a cylinder primitive
    cCreateArrow(mesh_arrow_x,       // mesh
                 1.0,              // length
                 0.01,             // radiusShaft
                 0.1,             // lengthTip
                 0.03,              // radiusTip
                 false,            // tipsAtBothExtremities?
                 32,               // numSides
                 cVector3d(1,0,0),  // direction
                 cVector3d(0,0,0)  // position
            //cColorf(R,G,B,alpha) // colour
    );


    // set material color
    mesh_arrow_x->m_material->setRed();

    // set haptic properties
    mesh_arrow_x->m_material->setStiffness(0.5 * maxStiffness);

    // build collision detection tree
    mesh_arrow_x->createAABBCollisionDetector(gripper_toolRadius);

    ////////////////////////////////////////////////////////////////////////
    // create a virtual mesh
    mesh_arrow_y = new cMesh();

    // add object to world
    world->addChild(mesh_arrow_y);

    // build mesh using a cylinder primitive
    cCreateArrow(mesh_arrow_y,       // mesh
                 1.0,              // length
                 0.01,             // radiusShaft
                 0.1,             // lengthTip
                 0.03,              // radiusTip
                 false,            // tipsAtBothExtremities?
                 32,               // numSides
                 cVector3d(0,1,0),  // direction
                 cVector3d(0,0,0)  // position
            //cColorf(R,G,B,alpha) // colour
    );


    // set material color
    mesh_arrow_y->m_material->setGreen();

    // set haptic properties
    mesh_arrow_y->m_material->setStiffness(0.5 * maxStiffness);

    // build collision detection tree
    mesh_arrow_y->createAABBCollisionDetector(gripper_toolRadius);


    ////////////////////////////////////////////////////////////
    // object mesh
    ///////////////////////////////////////////////////////////
    mesh_obj = new cMesh;
    // add object to world
    world->addChild(mesh_obj);
    cCreateBox(mesh_obj,
                0.3,
                0.3,
                0.3);

//    cCreateCylinder()

    // set material color
    mesh_obj->m_material->setGreen();
    // set haptic properties
//    mesh_obj->m_material->setStiffness(0.5 * maxStiffness);
    mesh_obj->m_material->setStiffness(1.0);
    // build collision detection tree
    mesh_obj->createAABBCollisionDetector(gripper_toolRadius);
    mesh_obj->setTransparencyLevel(0.8);

    mesh_obj->m_texture = cTexture2d::create();
    fileload = mesh_obj->m_texture->loadFromFile(RESOURCE_PATH("../../../src/chai/external/chai3d-3.2.0/bin/resources/images/whitefoam.jpg"));
    if (fileload) {
        cout << "Texture map image Found" << endl;
    }
    else {
        std::cout << "Error0 - Texture image failed to load correctly." << endl;
#if defined(_MSVC)
        fileload = mesh_obj->m_texture->loadFromFile("../../../src/chai/external/chai3d-3.2.0/bin/resources/images/spheremap-3.jpg");
#endif
#if defined(DEBUG_MODE)
        fileload = mesh_obj->m_texture->loadFromFile(RESOURCE_PATH("../../../../external/chai3d-3.2.0/bin/resources/images/whitefoam.jpg"));
#endif
        if (fileload) {
            cout << "Texture map image Found" << endl;
        }
    }
    if (!fileload)
    {
        std::cout << "Error1 - Texture image failed to load correctly." << endl;
        close();
        return (-1);
    }


//    object0->m_texture->setSphericalMappingEnabled(true);
    mesh_obj->setUseTexture(true);
    mesh_obj->m_material->setWhite();



    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    labelRates->m_fontColor.setBlack();
    camera->m_frontLayer->addChild(labelRates);

    // create a background
    background = new cBackground();
    camera->m_backLayer->addChild(background);

    // set background properties
    background->setCornerColors(cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(0.9f, 0.9f, 0.9f),
                                cColorf(0.9f, 0.9f, 0.9f));






    double gripper_angle_test  = 0.0;






    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);

    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback1(window1, width1, height1);
    windowSizeCallback2(window2, width2, height2);

    // main graphic loop
    while ((!glfwWindowShouldClose(window1)) && (!glfwWindowShouldClose(window2)) && ros::ok())
    {
        ////////////////////////////////////////////////////////////////////////
        // RENDER WINDOW 1
        ////////////////////////////////////////////////////////////////////////

        // activate display context
        glfwMakeContextCurrent(window1);

        // get width and height of window
        glfwGetWindowSize(window1, &width1, &height1);

        // render graphics
        updateGraphics1();

        // swap buffers
        glfwSwapBuffers(window1);


        ////////////////////////////////////////////////////////////////////////
        // RENDER WINDOW 2
        ////////////////////////////////////////////////////////////////////////

        // activate display context
        glfwMakeContextCurrent(window2);

        // get width and height of window
        glfwGetWindowSize(window2, &width2, &height2);

        // render graphics
        updateGraphics2();

        // swap buffers
        glfwSwapBuffers(window2);


        ////////////////////////////////////////////////////////////////////////
        // FINALIZE
        ////////////////////////////////////////////////////////////////////////

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);


        msg.position[0] = msg.position[0] + increment0;
        msg.position[1] = msg.position[1] + increment1;
//        msg.position[2] = msg.position[2] + increment2;q
        if (msg.position[0] > 0.15 || msg.position[0] < -0.15) {
            increment0 = -increment0;
        }
        if (msg.position[1] > 0.2 || msg.position[1] < -0.2) {
            increment1 = -increment1;
        }
        if (msg.position[2] > 0.1 || msg.position[2] < -0.1) {
            increment1 = -increment1;
        }

        //publish
        jointState_publisher.publish(msg);

        geometry_msgs::TransformStamped T;
        try
        {
            T = tfBuffer.lookupTransform("world", "MTMR_wrist_roll_link", ros::Time(0));
            cVector3d positions(T.transform.translation.x, T.transform.translation.y, T.transform.translation.z);
//            gripper_tool->setLocalPos(positions);
//            gripper_tool->setDeviceLocalPos(positions);
            gripper_tool->setDeviceLocalPos(cVector3d(0.0,0.0,0.0));
            gripper_tool->setLocalPos(positions);

            cout << "---------- position -----------" << endl;
            cout << positions << endl;

            tf2::Quaternion quat_tf;

            tf2::convert(T.transform.rotation , quat_tf);

            cout << "The rotation (Quaternion) is : "  << quat_tf.x() << "  " << quat_tf.y() << "  " << quat_tf.z() << "  " << quat_tf.w() << endl;

            tf2::Matrix3x3 rotations(quat_tf);
            cout << "--------- Convert to rotation matrix ----------" << endl;
            cout << rotations.getRow(0).x() << " " << rotations.getRow(0).y() << " " << rotations.getRow(0).z() << endl;
            cout << rotations.getRow(1).x() << " " << rotations.getRow(1).y() << " " << rotations.getRow(1).z() << endl;
            cout << rotations.getRow(2).x() << " " << rotations.getRow(2).y() << " " << rotations.getRow(2).z() << endl;
            cout << endl;

            cMatrix3d rotation_mat(rotations.getRow(0).x(), rotations.getRow(0).y(), rotations.getRow(0).z(),
                                   rotations.getRow(1).x(), rotations.getRow(1).y(), rotations.getRow(1).z(),
                                   rotations.getRow(2).x(), rotations.getRow(2).y(), rotations.getRow(2).z());

            cMatrix3d test_rot_mat(1.0, 0.0, 0.0,
                                   0.0, 1.0, 0.0,
                                   0.0, 0.0, 1.0);

            cout << "--------- test ROT matrix ----------" << endl;
            cout << test_rot_mat.getRow(0).x() << " " << test_rot_mat.getRow(0).y() << " " << test_rot_mat.getRow(0).z() << endl;
            cout << test_rot_mat.getRow(1).x() << " " << test_rot_mat.getRow(1).y() << " " << test_rot_mat.getRow(1).z() << endl;
            cout << test_rot_mat.getRow(2).x() << " " << test_rot_mat.getRow(2).y() << " " << test_rot_mat.getRow(2).z() << endl;
            cout << endl;

            test_rot_mat.rotateAboutGlobalAxisDeg(cVector3d(1.0, 0.0, 0.0), 90.0);

            cout << "--------- test ROT matrix ----------" << endl;
            cout << test_rot_mat.getRow(0).x() << " " << test_rot_mat.getRow(0).y() << " " << test_rot_mat.getRow(0).z() << endl;
            cout << test_rot_mat.getRow(1).x() << " " << test_rot_mat.getRow(1).y() << " " << test_rot_mat.getRow(1).z() << endl;
            cout << test_rot_mat.getRow(2).x() << " " << test_rot_mat.getRow(2).y() << " " << test_rot_mat.getRow(2).z() << endl;
            cout << endl;

            rotation_mat.mul(test_rot_mat);

            cout << "--------- After transform ----------" << endl;
            cout << rotation_mat.getRow(0).x() << " " << rotation_mat.getRow(0).y() << " " << rotation_mat.getRow(0).z() << endl;
            cout << rotation_mat.getRow(1).x() << " " << rotation_mat.getRow(1).y() << " " << rotation_mat.getRow(1).z() << endl;
            cout << rotation_mat.getRow(2).x() << " " << rotation_mat.getRow(2).y() << " " << rotation_mat.getRow(2).z() << endl;
            cout << endl;

            gripper_tool->setDeviceLocalRot(rotation_mat);
//            gripper_tool->setDeviceLocalRot(rotation_mat);


//            gripper_tool->setGripperAngleDeg(40.0);

//            if (gripper_angle_test < 90.0){
//                gripper_angle_test = gripper_angle_test + 0.5;
//            }
//            else {
//                gripper_angle_test = 0.0;
//            }

            gripper_tool->getHapticDevice()->m_specifications.m_rightHand = true;
            gripper_tool->setGripperAngleDeg(gripper_angle);

            gripper_tool->setShowFrame(true);

//            shaft->setFrameSize(0.15, false);
//            shaft->setShowFrame(true, false);
//            caudier_L->setFrameSize(0.1, false);
//            caudier_R->setFrameSize(0.1, false);
//            caudier_L->setShowFrame(true, false);
//            caudier_R->setShowFrame(true, false);


//            gripper_tool->setShowFrame(true, true);

//            gripper_tool->m_hapticPointThumb->m_sphereProxy->setLocalPos(-0.1, 0.1, 0.0);
//            gripper_tool->m_hapticPointFinger->m_sphereProxy->setLocalPos(0.01, 0.1, 0.0);

            gripper_tool->m_hapticPointFinger->setShow(true, true);
            gripper_tool->m_hapticPointThumb->setShow(true, true);

            gripper_tool->setShowContactPoints(true, true, cColorf(0.5, 0.5, 0.5));

            cout << "The tool gripper angle we can see is : " << gripper_tool->getGripperAngleDeg() << endl;
            cout << "Object Collision show? : " << object0->getShowCollisionDetector() << endl;

        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
//        std::cout << T << std::endl;

        //read and update all the topics
        ros::spinOnce();
        //delay to achieve desired publishing rate
        loop_rate.sleep();
    }

    // close windows
    glfwDestroyWindow(window1);
    glfwDestroyWindow(window2);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return 0;
}

//------------------------------------------------------------------------------
// msg callbacks
//------------------------------------------------------------------------------
void tf_position_cb(const geometry_msgs::Vector3::ConstPtr & msg)
{
    cVector3d object0_pos = object0->getLocalPos();
    object0_pos(0) = msg->z;
    object0_pos(1) = msg->x;
    object0_pos(2) = msg->y;
    cout << object0_pos << endl;
    object0->setLocalPos(object0_pos(0),object0_pos(1),object0_pos(2));

}

void pose_subscriber_cb(const geometry_msgs::PoseStamped::ConstPtr & msg){
    geometry_msgs::PoseStamped cur_pose = *msg;

    double scaleFactor = 3.0;
    tool->setDeviceGlobalPos(cVector3d(scaleFactor*cur_pose.pose.position.x,
                                       scaleFactor*cur_pose.pose.position.y,
                                       scaleFactor*cur_pose.pose.position.z));

    ///////////////////////////////////////////////////////////
    // robot pose display
    ///////////////////////////////////////////////////////////
    cout << "++++Robot position get++++ \n" << endl;
    cout << "current Cart position is : \n" << endl;
    cout << "       x : " << msg->pose.position.x << "\n" << endl;
    cout << "       y : " << msg->pose.position.y << "\n" << endl;
    cout << "       z : " << msg->pose.position.z << "\n" << endl;
    cout << "-------------------------------------" << endl;

}

//------------------------------------------------------------------------------
// CALLBACK WHEN CHANGING WINDOW SIZE
//------------------------------------------------------------------------------
void windowSizeCallback1(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width1  = a_width;
    height1 = a_height;
}
//------------------------------------------------------------------------------
void windowSizeCallback2(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width2  = a_width;
    height2 = a_height;
}

//------------------------------------------------------------------------------
// ERROR CALLBACK
//------------------------------------------------------------------------------
void errorCallback(int a_error, const char* a_description)
{
    cout << "Error!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl << endl;
    cout << "Error: " << a_description << endl;
    cout << "here!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl << endl;
}

//------------------------------------------------------------------------------
// KEY CALLBACK
//------------------------------------------------------------------------------
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    cVector3d camera_position;
    cVector3d upward_direction(0.0, 0.0, 1.0);
    cVector3d camera_look_at_position(0.25, -0.3645, 0.8712);
//    cVector3d camera_look_at_position(0.0, -0.0, 0.0);

    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

        // option - exit
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

        // option - up
    else if (a_key == GLFW_KEY_E)
    {
        camera_position = camera->getGlobalPos() + cVector3d(0.0, 0.0, 0.01);
        camera->set(camera_position,             // camera position (eye)
                    camera_look_at_position,    // look-at position (target)
                    upward_direction);         // direction of the (up) vector
    }

        // option - down
    else if (a_key == GLFW_KEY_Z)
    {
        camera_position = camera->getGlobalPos() + cVector3d(0.0, 0.0, -0.01);
        camera->set(camera_position,             // camera position (eye)
                    camera_look_at_position,    // look-at position (target)
                    upward_direction);         // direction of the (up) vector
    }

        // option - left
    else if (a_key == GLFW_KEY_A)
    {
        camera_position = camera->getGlobalPos() + cVector3d(-0.01, 0.0, 0.0);
        camera->set(camera_position,             // camera position (eye)
                    camera_look_at_position,    // look-at position (target)
                    upward_direction);         // direction of the (up) vector
    }

        // option - right
    else if (a_key == GLFW_KEY_D)
    {
        camera_position = camera->getGlobalPos() + cVector3d(0.01, 0.0, 0.0);
        camera->set(camera_position,             // camera position (eye)
                    camera_look_at_position,    // look-at position (target)
                    upward_direction);         // direction of the (up) vector
    }

        // option - away
    else if (a_key == GLFW_KEY_W)
    {
        camera_position = camera->getGlobalPos() + cVector3d(0.0, 0.01, 0.0);
        camera->set(camera_position,             // camera position (eye)
                    camera_look_at_position,    // look-at position (target)
                    upward_direction);         // direction of the (up) vector
    }

        // option - close
    else if (a_key == GLFW_KEY_X)
    {
        camera_position = camera->getGlobalPos() + cVector3d(0.0, -0.01, 0.0);
        camera->set(camera_position,             // camera position (eye)
                    camera_look_at_position,    // look-at position (target)
                    upward_direction);         // direction of the (up) vector
    }








        // option - up
    else if (a_key == GLFW_KEY_UP)
    {
        cVector3d object0_pos = object0->getLocalPos();
        object0_pos(1) = object0_pos(1) + 0.01;
        cout << object0_pos << endl;
        object0->setLocalPos(object0_pos(0),object0_pos(1),object0_pos(2));

        cVector3d mesh_obj_pos = mesh_obj->getGlobalPos();
        mesh_obj_pos(1) = mesh_obj_pos(1) + 0.01;
        mesh_obj->setLocalPos(mesh_obj_pos(0),mesh_obj_pos(1),mesh_obj_pos(2));

        cVector3d gripper_tool_pos = gripper_tool->getDeviceGlobalPos();
        gripper_tool_pos(0) = gripper_tool_pos(0) - 0.01;
        gripper_tool->setDeviceGlobalPos(gripper_tool_pos);
    }

        // option - down
    else if (a_key == GLFW_KEY_DOWN)
    {
        cVector3d object0_pos = object0->getLocalPos();
        object0_pos(1) = object0_pos(1) - 0.01;
        cout << object0_pos << endl;
        object0->setLocalPos(object0_pos(0),object0_pos(1),object0_pos(2));

        cVector3d mesh_obj_pos = mesh_obj->getGlobalPos();
        mesh_obj_pos(1) = mesh_obj_pos(1) - 0.01;
        mesh_obj->setLocalPos(mesh_obj_pos(0),mesh_obj_pos(1),mesh_obj_pos(2));
    }

        // option - left
    else if (a_key == GLFW_KEY_LEFT)
    {
        cVector3d object0_pos = object0->getLocalPos();
        object0_pos(0) = object0_pos(0) - 0.01;
        cout << object0_pos << endl;
        object0->setLocalPos(object0_pos(0),object0_pos(1),object0_pos(2));

        cVector3d mesh_obj_pos = mesh_obj->getGlobalPos();
        mesh_obj_pos(0) = mesh_obj_pos(0) - 0.01;
        mesh_obj->setLocalPos(mesh_obj_pos(0),mesh_obj_pos(1),mesh_obj_pos(2));
    }

        // option - right
    else if (a_key == GLFW_KEY_RIGHT)
    {
        cVector3d object0_pos = object0->getLocalPos();
        object0_pos(0) = object0_pos(0) + 0.01;
        cout << object0_pos << endl;
        object0->setLocalPos(object0_pos(0),object0_pos(1),object0_pos(2));

        cVector3d mesh_obj_pos = mesh_obj->getGlobalPos();
        mesh_obj_pos(0) = mesh_obj_pos(0) + 0.01;
        mesh_obj->setLocalPos(mesh_obj_pos(0),mesh_obj_pos(1),mesh_obj_pos(2));
    }

        // option - away
    else if (a_key == GLFW_KEY_P)
    {
        cVector3d object0_pos = object0->getLocalPos();
        object0_pos(2) = object0_pos(2) + 0.01;
        cout << object0_pos << endl;
        object0->setLocalPos(object0_pos(0),object0_pos(1),object0_pos(2));

        cVector3d mesh_obj_pos = mesh_obj->getGlobalPos();
        mesh_obj_pos(2) = mesh_obj_pos(2) + 0.01;
        mesh_obj->setLocalPos(mesh_obj_pos(0),mesh_obj_pos(1),mesh_obj_pos(2));
    }

        // option - close
    else if (a_key == GLFW_KEY_L)
    {
        cVector3d object0_pos = object0->getLocalPos();
        object0_pos(2) = object0_pos(2) - 0.01;
        cout << object0_pos << endl;
        object0->setLocalPos(object0_pos(0),object0_pos(1),object0_pos(2));

        cVector3d mesh_obj_pos = mesh_obj->getGlobalPos();
        mesh_obj_pos(2) = mesh_obj_pos(2) - 0.01;
        mesh_obj->setLocalPos(mesh_obj_pos(0),mesh_obj_pos(1),mesh_obj_pos(2));
    }







    else if (a_key == GLFW_KEY_I)
    {
        if ( (gripper_angle >= 0.0) && (gripper_angle < gripper_angle_lim_max) ) {
            gripper_angle = gripper_angle + 1.0;
        }
        else if (gripper_angle >= gripper_angle_lim_max) {
            gripper_angle = gripper_angle_lim_max;
        }
        caudier_L->rotateExtrinsicEulerAnglesDeg(gripper_angle, 0.0, 90.0, C_EULER_ORDER_ZXY);
        caudier_R->rotateExtrinsicEulerAnglesDeg(gripper_angle, 0.0, -90.0, C_EULER_ORDER_ZXY);
    }
    else if (a_key == GLFW_KEY_O)
    {
        if ( (gripper_angle > 0.0) && (gripper_angle <= gripper_angle_lim_max) ) {
            gripper_angle = gripper_angle - 1.0;
        }
        else if (gripper_angle <= 0.0) {
            gripper_angle = 0.0;
        }
        caudier_L->rotateExtrinsicEulerAnglesDeg(gripper_angle, 0.0, 90.0, C_EULER_ORDER_ZXY);
        caudier_R->rotateExtrinsicEulerAnglesDeg(gripper_angle, 0.0, -90.0, C_EULER_ORDER_ZXY);
    }

}



//------------------------------------------------------------------------------
// UPDATE GRAPHICS
//------------------------------------------------------------------------------
void updateGraphics1(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////
    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
                        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");
    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width1 - labelRates->getWidth())), 15);

    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE 1
    /////////////////////////////////////////////////////////////////////
    // update shadow maps (if any)
    world->updateShadowMaps(false, false);
    // render world
    camera->renderView(width1, height1, C_STEREO_LEFT_EYE);
    // wait until all GL commands are completed
    glFinish();
    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//------------------------------------------------------------------------------
void updateGraphics2(void)
{
    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE 2
    /////////////////////////////////////////////////////////////////////
    // update shadow maps (if any)
    world->updateShadowMaps(false, false);
    // render world
    camera->renderView(width2, height2, C_STEREO_RIGHT_EYE);
    // wait until all GL commands are completed
    glFinish();
    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}


//------------------------------------------------------------------------------
// UPDATE HAPTICS
//------------------------------------------------------------------------------
void updateHaptics(void)
{
    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;


//    ///////////////////////////////////////////
//    // Init tf listener
//    ///////////////////////////////////////////
//    tf2_ros::Buffer tfBuffer;
//    tf2_ros::TransformListener tfListener(tfBuffer);


    // main haptic simulation loop
    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////////
        // HAPTIC RENDERING
        /////////////////////////////////////////////////////////////////////////

        // signal frequency counter
        freqCounterHaptics.signal(1);

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
//        tool->updateFromDevice();
        // update position and orientation of tool
//        gripper_tool->updateFromDevice();


//        geometry_msgs::TransformStamped T;
//        try
//        {
//            T = tfBuffer.lookupTransform("world", "MTMR_wrist_roll_link", ros::Time(0));
//            cVector3d positions(T.transform.translation.x, T.transform.translation.y, T.transform.translation.z);
//            gripper_tool->setLocalPos(positions);
//
//            cout << "---------- position -----------" << endl;
//            cout << positions << endl;
//
//            tf2::Quaternion quat_tf;
//
//            tf2::convert(T.transform.rotation , quat_tf);
//
//            cout << "The rotation (Quaternion) is : "  << quat_tf.x() << "  " << quat_tf.y() << "  " << quat_tf.z() << "  " << quat_tf.w() << endl;
//
//            tf2::Matrix3x3 rotations(quat_tf);
//            cout << "--------- Convert to rotation matrix ----------" << endl;
//            cout << rotations.getRow(0).x() << " " << rotations.getRow(0).y() << " " << rotations.getRow(0).z() << endl;
//            cout << rotations.getRow(1).x() << " " << rotations.getRow(1).y() << " " << rotations.getRow(1).z() << endl;
//            cout << rotations.getRow(2).x() << " " << rotations.getRow(2).y() << " " << rotations.getRow(2).z() << endl;
//            cout << endl;
//
//            cMatrix3d rotation_mat(rotations.getRow(0).x(), rotations.getRow(0).y(), rotations.getRow(0).z(),
//                                   rotations.getRow(1).x(), rotations.getRow(1).y(), rotations.getRow(1).z(),
//                                   rotations.getRow(2).x(), rotations.getRow(2).y(), rotations.getRow(2).z());
//
//            rotation_mat.mul(cMatrix3d(1.0, 0.0, 0.0,
//                                       -0.0, 1.0, 0.0,
//                                       0.0, 0.0, 1.0));
//            cMatrix3d test_rot_mat;
//            test_rot_mat.rotateAboutGlobalAxisDeg(cVector3d(1.0, 0.0, 0.0), 90.0);
//
//            cout << "--------- test ROT matrix ----------" << endl;
//            cout << test_rot_mat.getRow(0).x() << " " << test_rot_mat.getRow(0).y() << " " << test_rot_mat.getRow(0).z() << endl;
//            cout << test_rot_mat.getRow(1).x() << " " << test_rot_mat.getRow(1).y() << " " << test_rot_mat.getRow(1).z() << endl;
//            cout << test_rot_mat.getRow(2).x() << " " << test_rot_mat.getRow(2).y() << " " << test_rot_mat.getRow(2).z() << endl;
//            cout << endl;
//
//            rotation_mat.mul(test_rot_mat);
//
//            cout << "--------- After transform ----------" << endl;
//            cout << rotation_mat.getRow(0).x() << " " << rotation_mat.getRow(0).y() << " " << rotation_mat.getRow(0).z() << endl;
//            cout << rotation_mat.getRow(1).x() << " " << rotation_mat.getRow(1).y() << " " << rotation_mat.getRow(1).z() << endl;
//            cout << rotation_mat.getRow(2).x() << " " << rotation_mat.getRow(2).y() << " " << rotation_mat.getRow(2).z() << endl;
//            cout << endl;
//
//
//            gripper_tool->setDeviceLocalRot(rotation_mat);
////            gripper_tool->setDeviceLocalRot(rotation_mat);
//
//            gripper_tool->setGripperAngleDeg(50.0);
//
////            shaft->setFrameSize(0.15, false);
////            shaft->setShowFrame(true, false);
////            caudier_L->setFrameSize(0.1, false);
////            caudier_R->setFrameSize(0.1, false);
////            caudier_L->setShowFrame(true, false);
////            caudier_R->setShowFrame(true, false);
//
//            gripper_tool->setShowFrame(true, true);
//
//            gripper_tool->m_hapticPointThumb->m_sphereProxy->setLocalPos(-0.01, 0.1, 0.0);
//            gripper_tool->m_hapticPointFinger->m_sphereProxy->setLocalPos(0.01, 0.1, 0.0);
//            gripper_tool->m_hapticPointFinger->setShow(true, true);
//
////            gripper_tool->get
//
//            cout << "The gripper angle we can see is : " << gripper_tool->getGripperAngleDeg() << endl;
//
//        }
//        catch (tf2::TransformException &ex)
//        {
//            ROS_WARN("%s", ex.what());
//            ros::Duration(1.0).sleep();
//            continue;
//        }
////        std::cout << T << std::endl;
//
//        //read and update all the topics
////        ros::spinOnce();




//        // check if a contact event has occurred at contact point
//        if (gripper_tool->m_hapticPointFinger->getNumCollisionEvents() > 0)
//        {
//            // get contact event
//            cCollisionEvent* collisionEvent = gripper_tool->m_hapticPointFinger->getCollisionEvent(0);
//            // get object from contact event
//            cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
//            cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
//            cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
//
//
//        }
//
//        gripper_tool->setShowFrame(true, true);
//
//        cout << "The global force is : " << gripper_tool->getDeviceGlobalForce() << endl;
//        cout << "The local force is : " << gripper_tool->getDeviceLocalForce() << endl;
//        cout << "The gripper force is : " << gripper_tool->getGripperForce() << endl;
//
//        cout << "The last computed force for the Thumb is : " << gripper_tool->m_hapticPointThumb->getLastComputedForce() << endl;
//        cout << "The last computed force for the Finger is : " << gripper_tool->m_hapticPointFinger->getLastComputedForce() << endl;
//
////        gripper_tool->m_hapticPointThumb->m_sphereGoal->rotateAboutGlobalAxisDeg(cVector3d(0.0,0.0,1.0), 45.0);
////        gripper_tool->setLocalPos(1.0, 1.0, 1.0);
//
//
//        cout << "gripper_tool global pos is : " << gripper_tool->getGlobalPos() << endl;
//        cout << "gripper_tool local pos is : " << gripper_tool->getLocalPos() << endl;
//        cout << "gripper_tool device global pos is : " << gripper_tool->getDeviceGlobalPos() << endl;
//        cout << "gripper_tool device local pos is : " << gripper_tool->getDeviceLocalPos() << endl;
//
//
//
//
//
//        force_line->setLineWidth(2.0);
//        force_line->m_pointA = shaft->getGlobalPos();
//        cVector3d force = gripper_tool->getDeviceGlobalForce();
//        force(0) = 5.0*force(0);
//        force(1) = 5.0*force(1);
//        force(2) = 5.0*force(2);
//        if ((force(0) != 0) || (force(1) != 0) || (force(2) != 0)) {
//            force_line->m_pointB = shaft->getGlobalPos() + force;
//        }
//        else {
//            force_line->m_pointB = shaft->getGlobalPos();
//        }
//        cout << "force is : " << force << endl;

        cout << "collition event 0 global position is  : " << gripper_tool->m_hapticPointThumb->getCollisionEvent(0)->m_globalPos << endl;
        cout << "collition event 0 normal direction is : " << gripper_tool->m_hapticPointThumb->getCollisionEvent(0)->m_globalNormal << endl;
        cCollisionRecorder a_recorder;
        cCollisionSettings a_settings;
//        cVector3d a_segmentPointA(0,0,0);
//        cVector3d a_segmentPointB(0,0,0);
        cVector3d a_segmentPointA;
        cVector3d a_segmentPointB;
        cout << mesh_obj->getCollisionDetector()->computeCollision(gripper_tool->m_hapticPointThumb->m_sphereProxy, a_segmentPointA, a_segmentPointB, a_recorder, a_settings) << endl;



        // compute interaction forces
        gripper_tool->computeInteractionForces();
        // send forces to haptic device
        gripper_tool->applyToDevice();
    }

    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
// CLOSE FUNCTION
//------------------------------------------------------------------------------
void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    hapticDevice->close();

    // delete resources
    delete [] (hapticsThread);
    delete [] (world);
    delete [] (handler);
//    delete [] camera;
//    delete [] light;
//    delete [] object0;
//    delete [] tool;
//    delete [] background;
//    delete [](labelRates);
//    delete [](window1);
//    delete [](window2);
}

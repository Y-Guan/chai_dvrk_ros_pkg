//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.2.0 $Rev: 2007 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "chai3d.h"
//---------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//---------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------
//#include "../ODE/src/CODE.h"
#include "../external/chai3d-3.2.0/modules/ODE/src/CODE.h"
//---------------------------------------------------------------------------

#include <iostream>
#include <ostream>
#include <objects.h>


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
#include <stdlib.h>
#include <stdio.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>
#include <joints/joint.h>
#include <math.h>

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;



//------------------------------------------------------------------------------
// DECLARED CHAI3D VARIABLES
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
// a virtual object
cMultiMesh* shaft;
cMultiMesh* caudier_L;
cMultiMesh* caudier_R;
cShapeLine* force_line;

cMesh* ground;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
//cGenericHapticDevicePtr hapticDevice;
shared_ptr<cGenericHapticDevice> hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;
cToolGripper* gripper_tool;

// a colored background
cBackground* background;

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;



//---------------------------------------------------------------------------
// ODE MODULE VARIABLES
//---------------------------------------------------------------------------

// ODE world
cODEWorld* ODEWorld;

// ODE object
cODEGenericBody* ODEGPlane_Base;
cODEGenericBody* ODEGPlane_Back;
cODEGenericBody* ODEGPlane_Left;
cODEGenericBody* ODEGPlane_Right;

cODEGenericBody* ODECaudier_L;
cODEGenericBody* ODECaudier_R;
cODEGenericBody* ODECaudier_shaft;
cODEGenericBody* ODECaudier_base;
cODEGenericBody* m_ODEBody0;
cODEGenericBody* m_ODEBody1;
cODEGenericBody* m_ODEBody2;

// a pointer the ODE object grasped by the tool
cODEGenericBody* graspObject;

// grasp position is respect to object
cVector3d graspPos;

// is grasp currently active?
bool graspActive = false;

// a small line used to display a grasp
cShapeLine* graspLine;

dJointID hinge_id;
dJointID hinge_id2;
dJointID fixed_id;


//---------------------------------------------------------------------------
// GENERAL VARIABLES
//---------------------------------------------------------------------------

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
GLFWwindow* window = NULL;
int width = 0;
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

// root resource path
string resourceRoot;


//------------------------------------------------------------------------------
// Declared local variables
//------------------------------------------------------------------------------
double gripper_angle = 0.0;
double gripper_angle_lim_max = 45.0;
cVector3d camera_look_at_position(0.2, 0.2, 0.5);
cVector3d desired_positions(0.0, 0.0, 0.0);
cMatrix3d rotation_mat1;
cMatrix3d rotation_mat2;
bool initialised;

double desired_yaw = 0.0;
double desired_roll = 0.0;
double desired_pitch = 0.0;

double previous_yaw_Error = 0.0;
double previous_pitch_Error = 0.0;
double previous_roll_Error = 0.0;

double integral_yaw = 0.0;
double integral_pitch = 0.0;
double integral_roll = 0.0;

tf2Scalar current_yaw;
tf2Scalar current_pitch;
tf2Scalar current_roll;

// a scope to monitor the yaw angles
cScope* scope_yaw;
// a scope to monitor the pitch angles
cScope* scope_pitch;
// a scope to monitor the roll angles
cScope* scope_roll;

dReal desired_quat[4];


//------------------------------------------------------------------------------
// DECLARED MACROS
//------------------------------------------------------------------------------

// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())

// toggle when debugging in CLion / Or just ignore this macro
#define DEBUG_MODE


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback for tf listener
void tf_position_cb(const geometry_msgs::Vector3::ConstPtr & msg);

void pose_subscriber_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

void gripper_angle_subscriber_cb(const sensor_msgs::JointState::ConstPtr &msg);

// callback when the window1 display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene1
void updateGraphics(void);

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

    /* ######################################################
    Create a topic subscriber:
     - topic name: /dvrk/MTMR/position_cartesian_current
     - message type: geometry_msgs::PoseStampedConstPtr
     - queue size: 10 (set to high if sending rate is high)
    #######################################################*/
    ros::Subscriber position_subscriber = node_obj.subscribe<geometry_msgs::PoseStamped>("/dvrk/MTMR/position_cartesian_current", 10, pose_subscriber_cb);

    /* ######################################################
    Create a topic subscriber:
     - topic name: /dvrk/MTMR/state_gripper_current
     - message type: sensor_msgs/JointState
     - queue size: 10 (set to high if sending rate is high)
    #######################################################*/
    ros::Subscriber gripper_angle_subscriber = node_obj.subscribe<sensor_msgs::JointState>("/dvrk/MTMR/state_gripper_current", 10, gripper_angle_subscriber_cb);

    // set the frequency of sending data
    ros::Rate loop_rate(60);



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



    //-----------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //-----------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    glfwSetErrorCallback(errorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int w = 0.8 * mode->height;
    int h = 0.5 * mode->height;
    int x = 0.5 * (mode->width - w);
    int y = 0.5 * (mode->height - h);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // set active stereo mode
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glfwWindowHint(GLFW_STEREO, GL_TRUE);
    }
    else
    {
        glfwWindowHint(GLFW_STEREO, GL_FALSE);
    }

    // create display context
    window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window, &width, &height);

    // set position of window
    glfwSetWindowPos(window, x, y);

    // set key callback
    glfwSetKeyCallback(window, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(window);

    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);

    // initialize GLEW library
#ifdef GLEW_VERSION
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
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

    world->setFrameSize(0.1);
    world->setShowFrame(true, true);


    //--------------------------------------------------------------------------
    // CAMERA
    //--------------------------------------------------------------------------

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
//    cVector3d camera_look_at_position(0.2, 0.0, 0.0);
    cVector3d camera_position(0.2, -1.3, 0.7);
//    cVector3d camera_position(0.5, -1.0, 0.5);
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

    // get access to the first available haptic device
//    shared_ptr<cGenericHapticDevice> hapticDevice;
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // emulate button if device has a force gripper
    hapticDevice->setEnableGripperUserSwitch(true);


    ////////////////////////////////////////////////////////
    // Cursor tool
    ////////////////////////////////////////////////////////
    // create a tool (cursor) and insert into the world
    tool = new cToolCursor(world);
    world->addChild(tool);

    // connect the haptic device to the virtual tool
    tool->setHapticDevice(hapticDevice);

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
    double gripper_toolRadius = 0.02;
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

    gripper_tool->setFrameSize(0.25);
    gripper_tool->setShowFrame(true);
    gripper_tool->m_hapticPointThumb->m_sphereProxy->setShowFrame(true);
    gripper_tool->m_hapticPointFinger->m_sphereProxy->setShowFrame(true);
    gripper_tool->getHapticDevice()->m_specifications.m_rightHand = true;
    gripper_tool->setShowContactPoints(true,true);


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFontPtr font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    labelRates->m_fontColor.setBlack();
    camera->m_frontLayer->addChild(labelRates);


    //--------------------------------------------------------------------------
    // CREATE OBJECTS and ODE World
    //--------------------------------------------------------------------------

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // properties
    double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

    bool fileload;


    //////////////////////////////////////////////////////////////////////////
    // graspLine
    //////////////////////////////////////////////////////////////////////////

    // create a small white line that will be enabled every time the operator
    // grasps an object. The line indicated the connection between the
    // position of the tool and the grasp position on the object
    graspLine = new cShapeLine(cVector3d(0,0,0), cVector3d(0,0,0));
    world->addChild(graspLine);
    graspLine->m_colorPointA.set(1.0, 1.0, 1.0);
    graspLine->m_colorPointB.set(1.0, 1.0, 1.0);
    graspLine->setShowEnabled(false);


    //////////////////////////////////////////////////////////////////////////
    // ODE WORLD
    //////////////////////////////////////////////////////////////////////////

    // create an ODE world to simulate dynamic bodies
    ODEWorld = new cODEWorld(world);

    // add ODE world as a node inside world
    world->addChild(ODEWorld);

    // set some gravity
    ODEWorld->setGravity(cVector3d(0.0, 0.0, -9.81));
    // ODEWorld->setGravity(cVector3d(0.0, 0.0, 0.0));

    // define damping properties
    ODEWorld->setAngularDamping(0.002);
    ODEWorld->setLinearDamping(0.002);


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
    fileload = shaft->loadFromFile(RESOURCE_PATH("../../../src/chai/external/chai3d-3.2.0/bin/resources/models/endoscope/tool_wrist_caudier_link_1_shaft.stl"));

    if (!fileload)
    {
#if defined(DEBUG_MODE)
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
    shaft->scale(15.0);

    // use display list for faster rendering
    shaft->setUseDisplayList(true);

    // position object in scene
//    shaft->rotateExtrinsicEulerAnglesDeg(0.0, 0.0, 0.0, C_EULER_ORDER_XYZ);
    shaft->rotateExtrinsicEulerAnglesDeg(0.0, 90.0, 0.0, C_EULER_ORDER_XYZ);

    cout << "shaft transparency? : " << shaft->getUseTransparency() << endl;

    shaft->setShowFrame(true);





    //////////////////////////////////////////////////////////
    // Ground
    //////////////////////////////////////////////////////////
    ground = new cMesh();
    world->addChild(ground);

    cCreatePlane(ground, 1.5, 1.5);
    // position ground in world where the invisible ODE Base plane is located
    ground->setLocalPos(0.0, 0.0, 0.0);

//    ground->m_material->setGrayLevel(0.3);
    ground->m_material->setBlueRoyal();

    ground->m_material->setStiffness(1500);
    ground->m_material->setDynamicFriction(0.2);
    ground->m_material->setStaticFriction(0.2);
    ground->createAABBCollisionDetector(gripper_toolRadius);

    ground->setStiffness(1500);

    ground->setShowEnabled(true);
    ground->setTransparencyLevel(0.7);


    //////////////////////////////////////////////////////////////
    //
    //////////////////////////////////////////////////////////////
    // we now create 6 static walls to bound the workspace of our simulation
    ODEGPlane_Base = new cODEGenericBody(ODEWorld);
//    ODEGPlane_Back = new cODEGenericBody(ODEWorld);
//    ODEGPlane_Left = new cODEGenericBody(ODEWorld);
//    ODEGPlane_Right = new cODEGenericBody(ODEWorld);

    double size = 1.0;
    ODEGPlane_Base->createStaticPlane(cVector3d(0.0, 0.0,  0.0), cVector3d(0.0, 0.0 , 1.0));
//    ODEGPlane_Back->createStaticPlane(cVector3d(0.0, 0.0, -size), cVector3d(0.0, 0.0 , 1.0));
//    ODEGPlane_Left->createStaticPlane(cVector3d(0.0,  size, 0.0), cVector3d(0.0,-1.0, 0.0));
//    ODEGPlane_Right->createStaticPlane(cVector3d(0.1, 0.0, 0.0), cVector3d(0.0, 1.0, 0.0));
//    ODEGPlane4->createStaticPlane(cVector3d( size, 0.0, 0.0), cVector3d(-1.0,0.0, 0.0));
//    ODEGPlane5->createStaticPlane(cVector3d(-0.8 * size, 0.0, 0.0), cVector3d( 1.0,0.0, 0.0));


//    ODEGPlane_Right->m_material->setYellow();




    // create a new ODE object that is automatically added to the ODE world
    m_ODEBody0 = new cODEGenericBody(ODEWorld);
    cMesh* object0 = new cMesh();
    cCreateRing(object0, 0.03, 0.06, 12, 16);
    object0->createAABBCollisionDetector(toolRadius);

    // define some material properties for each cube
    cMaterial mat;
    mat.setBlueRoyal();
    mat.m_specular.set(0.0, 0.0, 0.0);
    mat.setStiffness(1500);
    mat.setDynamicFriction(0.2);
    mat.setStaticFriction(0.2);
    object0->setMaterial(mat);
    m_ODEBody0->setImageModel(object0);
    m_ODEBody0->createDynamicMesh();
    m_ODEBody0->setMass(0.05);
    m_ODEBody0->setLocalPos(0.06, 0.02, 0.30);
    m_ODEBody0->rotateAboutGlobalAxisDeg(cVector3d(0.0, 1.0, 0.0), 90.0);





    cMaterial matCaudier;
    matCaudier.setRedCrimson();
    matCaudier.setStiffness(1500);
    matCaudier.setDynamicFriction(0.2);
    matCaudier.setStaticFriction(0.2);



//    ODECaudier_shaft = new cODEGenericBody(ODEWorld);
//    cMultiMesh* ODECaudier_shaft_mesh = new cMultiMesh();
//
////    fileload = ODECaudier_shaft_mesh->loadFromFile(RESOURCE_PATH("../../../../external/chai3d-3.2.0/bin/resources/models/endoscope/tool_wrist_caudier_link_1_shaft.stl"));
//    fileload = ODECaudier_shaft_mesh->loadFromFile(RESOURCE_PATH("../../../../external/chai3d-3.2.0/bin/resources/models/endoscope/caudier_shaft2.stl"));
//    if (!fileload)
//    {
//#if defined(_MSVC)
//        fileload = gear1->loadFromFile("../../../bin/resources/models/gear/gear.3ds");
//#endif
//    }
//    ODECaudier_shaft_mesh->scale(10.0/1000000);
////    ODECaudier_shaft_mesh->scale(10.0);
//    ODECaudier_shaft_mesh->createAABBCollisionDetector(gripper_toolRadius);
//
//    ODECaudier_shaft_mesh->setMaterial(matCaudier, true);
//
//    ODECaudier_shaft->setImageModel(ODECaudier_shaft_mesh);
//
//    ODECaudier_shaft->createDynamicMesh();
//
//    ODECaudier_shaft->setMass(0.02);
//
//    ODECaudier_shaft->setLocalPos(0.1, -0.4, 0.12);

//    ODECaudier_shaft->rotateAboutGlobalAxisDeg(cVector3d(0.0, 1.0, 0.0), 90.0);


    ODECaudier_L = new cODEGenericBody(ODEWorld);
    cMultiMesh* ODECaudier_L_mesh = new cMultiMesh();

//    ODECaudier_shaft->addChild(ODECaudier_L);
    fileload = ODECaudier_L_mesh->loadFromFile(RESOURCE_PATH("../../../src/chai/external/chai3d-3.2.0/bin/resources/models/endoscope/caudier_fixed.stl"));
    if (!fileload)
    {
        cout << "here" << endl;
#if defined(_MSVC)
        fileload = gear1->loadFromFile("../../../bin/resources/models/gear/gear.3ds");
#endif
    }
    ODECaudier_L_mesh->scale(10.0);
    ODECaudier_L_mesh->createAABBCollisionDetector(gripper_toolRadius);

    ODECaudier_L_mesh->setMaterial(matCaudier, true);

    ODECaudier_L->setImageModel(ODECaudier_L_mesh);

    ODECaudier_L->createDynamicMesh();

    ODECaudier_L->setMass(0.06);

//    ODECaudier_L->setLocalPos(0.2, 0.4, 0.15+0.003+0.0165);

    ODECaudier_L->setLocalPos(0.2, 0.4, 0.15);

//    hinge_id = dJointCreateHinge(ODEWorld->m_ode_world, ODEWorld->m_ode_contactgroup);
//    dJointAttach(hinge_id, ODECaudier_shaft->m_ode_body, ODECaudier_L->m_ode_body);
//    cout << dJointGetNumBodies(hinge_id) << endl;




    ODECaudier_R = new cODEGenericBody(ODEWorld);
    cMultiMesh* ODECaudier_R_mesh = new cMultiMesh();

    fileload = ODECaudier_R_mesh->loadFromFile(RESOURCE_PATH("../../../src/chai/external/chai3d-3.2.0/bin/resources/models/endoscope/caudier_fixed.stl"));
    if (!fileload)
    {
#if defined(_MSVC)
        fileload = gear1->loadFromFile("../../../bin/resources/models/gear/gear.3ds");
#endif
    }
    ODECaudier_R_mesh->scale(10.0);
    ODECaudier_R_mesh->createAABBCollisionDetector(gripper_toolRadius);

    ODECaudier_R_mesh->setMaterial(matCaudier, true);

    ODECaudier_R->setImageModel(ODECaudier_R_mesh);

    ODECaudier_R->createDynamicMesh();

    ODECaudier_R->setMass(0.06);

    ODECaudier_R->rotateAboutGlobalAxisDeg(cVector3d(0.0, 1.0, 0.0), 180.0);

//    ODECaudier_R->setLocalPos(0.2, 0.4, 0.15-0.039/2);

    ODECaudier_R->setLocalPos(0.2, 0.4, 0.15);

    fixed_id = dJointCreateFixed(ODEWorld->m_ode_world, ODEWorld->m_ode_contactgroup);
    dJointAttach(fixed_id, ODECaudier_L->m_ode_body, ODECaudier_R->m_ode_body);





//    ODECaudier_base = new cODEGenericBody(ODEWorld);
//    cMultiMesh* ODECaudier_base_mesh = new cMultiMesh();
//
//    fileload = ODECaudier_base_mesh->loadFromFile(RESOURCE_PATH("../../../../external/chai3d-3.2.0/bin/resources/models/endoscope/tool_wrist_caudier_link_1.stl"));
//    if (!fileload)
//    {
//#if defined(_MSVC)
//        fileload = gear1->loadFromFile("../../../bin/resources/models/gear/gear.3ds");
//#endif
//    }
//    ODECaudier_base_mesh->scale(10.1);
//    ODECaudier_base_mesh->createAABBCollisionDetector(gripper_toolRadius);
//
//    ODECaudier_base_mesh->setMaterial(matCaudier, true);
//    ODECaudier_base_mesh->setMaterial(matCaudier, true);
//
//    ODECaudier_base->setImageModel(ODECaudier_base_mesh);
//
//    ODECaudier_base->createDynamicMesh();
//
//    ODECaudier_base->setMass(0.06);
//
//    ODECaudier_base->setLocalPos(0.0, -0.514, 0.1); //-0.5127
//
//    ODECaudier_base->rotateAboutGlobalAxisDeg(cVector3d(1.0, 0.0, 0.0), 90.0);
//
//    ODECaudier_base->setShowFrame(true);


    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------
    
    // create a scope to plot yaw angles
    scope_yaw = new cScope();
    camera->m_frontLayer->addChild(scope_yaw);
    scope_yaw->setLocalPos(20,20);
    scope_yaw->setSize(500,300);
    scope_yaw->setRange(-M_PI, M_PI);
    scope_yaw->setSignalEnabled(true, true, false, false);
    scope_yaw->setTransparencyLevel(0.6);
    scope_yaw->m_colorSignal0.setBlack();
    scope_yaw->m_colorSignal1.setYellowDarkKhaki();
    scope_yaw->setLineWidth(5.0);

    // create a scope to plot pitch angles
    scope_pitch = new cScope();
    camera->m_frontLayer->addChild(scope_pitch);
    scope_pitch->setLocalPos(620,20);
    scope_pitch->setSize(500,300);
    scope_pitch->setRange(-M_PI, M_PI);
    scope_pitch->setSignalEnabled(true, true, false, false);
    scope_pitch->setTransparencyLevel(0.6);
    scope_pitch->m_colorSignal0.setBlack();
    scope_pitch->m_colorSignal1.setYellowDarkKhaki();
    scope_pitch->setLineWidth(5.0);

    // create a scope to plot yaw angles
    scope_roll = new cScope();
    camera->m_frontLayer->addChild(scope_roll);
    scope_roll->setLocalPos(1220,20);
    scope_roll->setSize(500,300);
    scope_roll->setRange(-M_PI, M_PI);
    scope_roll->setSignalEnabled(true, true, false, false);
    scope_roll->setTransparencyLevel(0.6);
    scope_roll->m_colorSignal0.setBlack();
    scope_roll->m_colorSignal1.setYellowDarkKhaki();
    scope_roll->setLineWidth(5.0);


    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
        // get width and height of window
        glfwGetWindowSize(window, &width, &height);

        // render graphics
        updateGraphics();

        // swap buffers
        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);





        msg.position[0] = msg.position[0] + increment0;
        msg.position[1] = msg.position[1] + increment1;
//        msg.position[2] = msg.position[2] + increment2;
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
        // jointState_publisher.publish(msg);


        geometry_msgs::TransformStamped T;
        try
        {
            /*
            T = tfBuffer.lookupTransform("world", "MTMR_wrist_roll_link", ros::Time(0));
            positions = cVector3d(T.transform.translation.x, T.transform.translation.y, T.transform.translation.z);

//            cout << "---------- position -----------" << endl;
//            cout << positions << endl;

            tf2::Quaternion quat_tf;

            tf2::convert(T.transform.rotation , quat_tf);

//            cout << "The rotation (Quaternion) is : "  << quat_tf.x() << "  " << quat_tf.y() << "  " << quat_tf.z() << "  " << quat_tf.w() << endl;
//
            tf2::Matrix3x3 rotations(quat_tf);
//            cout << "--------- Convert to rotation matrix ----------" << endl;
//            cout << rotations.getRow(0).x() << " " << rotations.getRow(0).y() << " " << rotations.getRow(0).z() << endl;
//            cout << rotations.getRow(1).x() << " " << rotations.getRow(1).y() << " " << rotations.getRow(1).z() << endl;
//            cout << rotations.getRow(2).x() << " " << rotations.getRow(2).y() << " " << rotations.getRow(2).z() << endl;
//            cout << endl;
//
            rotation_mat1 = cMatrix3d(rotations.getRow(0).x(), rotations.getRow(0).y(), rotations.getRow(0).z(),
                                      rotations.getRow(1).x(), rotations.getRow(1).y(), rotations.getRow(1).z(),
                                      rotations.getRow(2).x(), rotations.getRow(2).y(), rotations.getRow(2).z());
            rotation_mat2 = cMatrix3d(rotations.getRow(0).x(), rotations.getRow(0).y(), rotations.getRow(0).z(),
                                      rotations.getRow(1).x(), rotations.getRow(1).y(), rotations.getRow(1).z(),
                                      rotations.getRow(2).x(), rotations.getRow(2).y(), rotations.getRow(2).z());

            cMatrix3d test_rot_mat(0.0, 0.0, 1.0,
                                   1.0, 0.0, 0.0,
                                   0.0, 1.0, 0.0);

            cMatrix3d test_rot_mat2(-0.0, 0.0, -1.0,
                                    -1.0, 0.0, -0.0,
                                    0.0, 1.0, 0.0);
//
//            cout << "--------- test ROT matrix ----------" << endl;
//            cout << test_rot_mat.getRow(0).x() << " " << test_rot_mat.getRow(0).y() << " " << test_rot_mat.getRow(0).z() << endl;
//            cout << test_rot_mat.getRow(1).x() << " " << test_rot_mat.getRow(1).y() << " " << test_rot_mat.getRow(1).z() << endl;
//            cout << test_rot_mat.getRow(2).x() << " " << test_rot_mat.getRow(2).y() << " " << test_rot_mat.getRow(2).z() << endl;
//            cout << endl;
//
//            test_rot_mat.rotateAboutGlobalAxisDeg(cVector3d(1.0, 0.0, 0.0), 90.0);
//
//            cout << "--------- test ROT matrix ----------" << endl;
//            cout << test_rot_mat.getRow(0).x() << " " << test_rot_mat.getRow(0).y() << " " << test_rot_mat.getRow(0).z() << endl;
//            cout << test_rot_mat.getRow(1).x() << " " << test_rot_mat.getRow(1).y() << " " << test_rot_mat.getRow(1).z() << endl;
//            cout << test_rot_mat.getRow(2).x() << " " << test_rot_mat.getRow(2).y() << " " << test_rot_mat.getRow(2).z() << endl;
//            cout << endl;
//
            rotation_mat1.mul(test_rot_mat);
            rotation_mat2.mul(test_rot_mat2);

            rotation_mat1.rotateAboutLocalAxisDeg(cVector3d(0.0, 0.0, 1.0), gripper_angle);
            rotation_mat2.rotateAboutLocalAxisDeg(cVector3d(0.0, 0.0, 1.0), gripper_angle);

//
//            cout << "--------- After transform ----------" << endl;
//            cout << rotation_mat.getRow(0).x() << " " << rotation_mat.getRow(0).y() << " " << rotation_mat.getRow(0).z() << endl;
//            cout << rotation_mat.getRow(1).x() << " " << rotation_mat.getRow(1).y() << " " << rotation_mat.getRow(1).z() << endl;
//            cout << rotation_mat.getRow(2).x() << " " << rotation_mat.getRow(2).y() << " " << rotation_mat.getRow(2).z() << endl;
//            cout << endl;
//
//            gripper_tool->setDeviceLocalRot(rotation_mat);




//            gripper_tool->setDeviceLocalRot(rotation_mat);


//            gripper_tool->setGripperAngleDeg(40.0);

//            if (gripper_angle_test < 90.0){
//                gripper_angle_test = gripper_angle_test + 0.5;
//            }
//            else {
//                gripper_angle_test = 0.0;
//            }

//            gripper_tool->getHapticDevice()->m_specifications.m_rightHand = true;
//            gripper_tool->setGripperAngleDeg(gripper_angle);
//
//            gripper_tool->setShowFrame(true);
//
//
//
//            gripper_tool->m_hapticPointFinger->setShow(true, true);
//            gripper_tool->m_hapticPointThumb->setShow(true, true);
//
//            gripper_tool->setShowContactPoints(true, true, cColorf(0.5, 0.5, 0.5));

            initialised = true;
            */

        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        //read and update all the topics
        ros::spinOnce();
        //delay to achieve desired publishing rate
        loop_rate.sleep();


    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return 0;


}


///////////////////////////////////////////////////////////////////////////////
/// \brief position_subscriber_cb
/// \param msg
///////////////////////////////////////////////////////////////////////////////
void pose_subscriber_cb(const geometry_msgs::PoseStamped::ConstPtr & msg){

    geometry_msgs::PoseStamped cur_pose = *msg;

	// compute position
	double scaleFactor = 1.5;
	// gripper_tool->setDeviceLocalPos(cVector3d(0.0, 0.0, 0.0));
    // gripper_tool->setLocalPos(cVector3d(scaleFactor*cur_pose.pose.position.x,
								// scaleFactor*cur_pose.pose.position.y,
								// scaleFactor*cur_pose.pose.position.z));

    desired_positions = cVector3d(scaleFactor*cur_pose.pose.position.x, scaleFactor*cur_pose.pose.position.y , scaleFactor*cur_pose.pose.position.z + 0.5);

    // cout << "++++Robot position get++++ \n" << endl;
    // cout << "current Cart position is : \n" << endl;
    // cout << "       x : " << msg->pose.position.x << "\n" << endl;
    // cout << "       y : " << msg->pose.position.y << "\n" << endl;
    // cout << "       z : " << msg->pose.position.z << "\n" << endl;
    // cout << "-------------------------------------" << endl;

	// compute orientation
	tf2::Quaternion quat_tf;
	tf2::convert(cur_pose.pose.orientation, quat_tf);
	cout << "The rotation (Quaternion) is : " << "x " << quat_tf.x() << " " << "y " << quat_tf.y() << " " << "z " << quat_tf.z() << " " << "w " << quat_tf.w() << endl;

    tf2::Quaternion quat_tf_preRot;
    quat_tf_preRot.setValue(0.5, 0.5, 0.5, 0.5);
    // quat_tf_preRot.set
    // quat_tf_preRot.setEulerZYX(-M_PI_2, 0.0, -M_PI_2);
    // quat_tf.operator*=(quat_tf_preRot);

    // quat_tf = quat_tf_preRot * quat_tf;

    

	// convert to rotation matrix
	tf2::Matrix3x3 rotations(quat_tf);

    tf2Scalar Yaw;
    tf2Scalar Pitch;
    tf2Scalar Roll;
    rotations.getEulerYPR(Yaw, Pitch, Roll);

    cout << Yaw << "  " << Pitch << "  " << Roll << endl;

    



    rotation_mat1 = cMatrix3d(rotations.getRow(0).x(), rotations.getRow(0).y(), rotations.getRow(0).z(),
                                rotations.getRow(1).x(), rotations.getRow(1).y(), rotations.getRow(1).z(),
                                rotations.getRow(2).x(), rotations.getRow(2).y(), rotations.getRow(2).z());
    rotation_mat2 = cMatrix3d(rotations.getRow(0).x(), rotations.getRow(0).y(), rotations.getRow(0).z(),
                                rotations.getRow(1).x(), rotations.getRow(1).y(), rotations.getRow(1).z(),
                                rotations.getRow(2).x(), rotations.getRow(2).y(), rotations.getRow(2).z());

    cMatrix3d test_rot_mat(0.0, 0.0, 1.0,
                            1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0);

    cMatrix3d test_rot_mat2(-0.0, 0.0, -1.0,
                            -1.0, 0.0, -0.0,
                            0.0, 1.0, 0.0);

    rotation_mat1.mul(test_rot_mat);
    rotation_mat2.mul(test_rot_mat2);

    tf2::Matrix3x3 rotation2;
    rotation2[0][0] = rotation_mat1.getRow(0).x();
    rotation2[0][1] = rotation_mat1.getRow(0).y();
    rotation2[0][2] = rotation_mat1.getRow(0).z();
    rotation2[1][0] = rotation_mat1.getRow(1).x();
    rotation2[1][1] = rotation_mat1.getRow(1).y();
    rotation2[1][2] = rotation_mat1.getRow(1).z();
    rotation2[2][0] = rotation_mat1.getRow(2).x();
    rotation2[2][1] = rotation_mat1.getRow(2).y();
    rotation2[2][2] = rotation_mat1.getRow(2).z();
    rotation2.getEulerYPR(Yaw, Roll, Pitch, 1);

    rotation_mat1.rotateAboutLocalAxisDeg(cVector3d(0.0, 0.0, 1.0), gripper_angle);
    rotation_mat2.rotateAboutLocalAxisDeg(cVector3d(0.0, 0.0, 1.0), gripper_angle);

    cMatrix3d desired_rot = rotation_mat1;

    desired_yaw = Yaw;
    desired_pitch = Pitch;
    desired_roll = Roll;

    initialised = true;

    cout << "desired Yaw Pitch Roll : " << desired_yaw << "  " << desired_pitch << "  " << desired_roll << endl;

    // quat_tf.setRPY(desired_yaw, desired_roll, desired_pitch);
    // quat_tf.setEulerZYX(desired_yaw, desired_roll, desired_pitch);

    desired_quat[0] = quat_tf.x();
    desired_quat[1] = quat_tf.y();
    desired_quat[2] = quat_tf.z();
    desired_quat[3] = quat_tf.w();

    // desired_quat[0] = quat_tf.z();
    // desired_quat[1] = quat_tf.y();
    // desired_quat[2] = quat_tf.x();
    // desired_quat[3] = quat_tf.w();


	// cMatrix3d rotation_mat(rotations.getRow(0).x(), rotations.getRow(0).y(), rotations.getRow(0).z(),
	// 					   rotations.getRow(1).x(), rotations.getRow(1).y(), rotations.getRow(1).z(),
	// 					   rotations.getRow(2).x(), rotations.getRow(2).y(), rotations.getRow(2).z());
	// rotation_mat.mul(cMatrix3d(1.0, 0.0, 0.0,
	// 						   0.0, 1.0, 0.0,
	// 						   0.0, 0.0, 1.0));

	// // rotate 90 degree about the x axis

	// //TODO!!!!!!!!!!!!	

	// cMatrix3d x90_rot_mat = cMatrix3d(1.0, 0.0, 0.0,
	// 						   		  0.0, 1.0, 0.0,
	// 						   		  0.0, 0.0, 1.0);

	// //cout << x90_rot_mat << endl;

	// x90_rot_mat.rotateAboutGlobalAxisDeg(cVector3d(1.0, 0.0, 0.0), 90.0);

	// rotation_mat.mul(x90_rot_mat);

	// cout << x90_rot_mat.getRow(0).x() << " " << x90_rot_mat.getRow(0).y() << " " << x90_rot_mat.getRow(0).z() << endl;
	// cout << x90_rot_mat.getRow(1).x() << " " << x90_rot_mat.getRow(1).y() << " " << x90_rot_mat.getRow(1).z() << endl;
	// cout << x90_rot_mat.getRow(2).x() << " " << x90_rot_mat.getRow(2).y() << " " << x90_rot_mat.getRow(2).z() << endl;

	// cout << rotation_mat.getRow(0).x() << " " << rotation_mat.getRow(0).y() << " " << rotation_mat.getRow(0).z() << endl;
	// cout << rotation_mat.getRow(1).x() << " " << rotation_mat.getRow(1).y() << " " << rotation_mat.getRow(1).z() << endl;
	// cout << rotation_mat.getRow(2).x() << " " << rotation_mat.getRow(2).y() << " " << rotation_mat.getRow(2).z() << endl;


	// // set the device (gripper tool proxy and goal) local rotation
	// // Rotate 90 degree about the ?????????? //TODO check!
	// gripper_tool->setDeviceLocalRot(rotation_mat);

	// cout << "caudier position " << caudier_L->getGlobalPos() << endl;

	// // show the tool proxy and goal
	// gripper_tool->m_hapticPointThumb->setShow(true, true);
	// gripper_tool->m_hapticPointFinger->setShow(true, true);


}

//////////////////////////////////////////////////////////////////////////////
/// \brief gripper_angle_subscriber_cb
/// \param msg
//////////////////////////////////////////////////////////////////////////////
void gripper_angle_subscriber_cb(const sensor_msgs::JointState::ConstPtr &msg)
{
    sensor_msgs::JointState cur_gripper_angle = *msg;
    gripper_angle = (cur_gripper_angle.position[0]/3.14) * 180.0;

}



//---------------------------------------------------------------------------
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width  = a_width;
    height = a_height;
}

//------------------------------------------------------------------------------
void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//---------------------------------------------------------------------------
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    cVector3d camera_position;
    cVector3d upward_direction(0.0, 0.0, 1.0);

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

        // option - toggle fullscreen
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

        // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_M)
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
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
//        camera_look_at_position = camera_look_at_position + cVector3d(-0.01, 0.0, 0.0);
        camera->set(camera_position,             // camera position (eye)
                    camera_look_at_position,    // look-at position (target)
                    upward_direction);         // direction of the (up) vector
    }

        // option - right
    else if (a_key == GLFW_KEY_D)
    {
        camera_position = camera->getGlobalPos() + cVector3d(0.01, 0.0, 0.0);
//        camera_look_at_position = camera_look_at_position + cVector3d(0.01, 0.0, 0.0);
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




        // tool forward
    else if (a_key == GLFW_KEY_UP)
    {

        cVector3d gripper_tool_pos = gripper_tool->getDeviceGlobalPos();
        gripper_tool_pos(1) = gripper_tool_pos(1) + 0.01;
        gripper_tool->setDeviceGlobalPos(gripper_tool_pos);

    }
        // tool back
    else if (a_key == GLFW_KEY_DOWN)
    {
        cVector3d gripper_tool_pos = gripper_tool->getDeviceGlobalPos();
        gripper_tool_pos(1) = gripper_tool_pos(1) - 0.01;
        gripper_tool->setDeviceGlobalPos(gripper_tool_pos);

    }
        // tool left
    else if (a_key == GLFW_KEY_LEFT)
    {
        cVector3d gripper_tool_pos = gripper_tool->getDeviceGlobalPos();
        gripper_tool_pos(0) = gripper_tool_pos(0) - 0.01;
        gripper_tool->setDeviceGlobalPos(gripper_tool_pos);

    }
        // tool right
    else if (a_key == GLFW_KEY_RIGHT)
    {
        cVector3d gripper_tool_pos = gripper_tool->getDeviceGlobalPos();
        gripper_tool_pos(0) = gripper_tool_pos(0) + 0.01;
        gripper_tool->setDeviceGlobalPos(gripper_tool_pos);

    }
        // tool up
    else if (a_key == GLFW_KEY_P)
    {
        cVector3d gripper_tool_pos = gripper_tool->getDeviceGlobalPos();
        gripper_tool_pos(2) = gripper_tool_pos(2) + 0.01;
        gripper_tool->setDeviceGlobalPos(gripper_tool_pos);

    }
        // tool down
    else if (a_key == GLFW_KEY_L)
    {
        cVector3d gripper_tool_pos = gripper_tool->getDeviceGlobalPos();
        gripper_tool_pos(2) = gripper_tool_pos(2) - 0.01;
        gripper_tool->setDeviceGlobalPos(gripper_tool_pos);

    }


    else if (a_key == GLFW_KEY_G)
    {
        if (gripper_tool->getUserSwitch(0) == false) {gripper_tool->setUserSwitch(0,true);}
        else {
            gripper_tool->setUserSwitch(0,false);
        }
    }

    else if (a_key == GLFW_KEY_I)
    {
        gripper_angle = gripper_angle - 1.0;
        gripper_tool->setGripperAngleDeg(gripper_angle);
    }

    else if (a_key == GLFW_KEY_O)
    {
        gripper_angle = gripper_angle + 1.0;
        gripper_tool->setGripperAngleDeg(gripper_angle);
    }


    else if (a_key == GLFW_KEY_F)
    {

        cVector3d ODE_L_local_pos = ODECaudier_L->getLocalPos();
        cout << ODE_L_local_pos.x() << " " << ODE_L_local_pos.y() << " " << ODE_L_local_pos.z() << endl;
//        ODECaudier_L->setLocalPos(ODE_L_local_pos.x()+0.001 , ODE_L_local_pos.y(), ODE_L_local_pos.z());

        fixed_id = dJointCreateFixed(ODEWorld->m_ode_world, ODEWorld->m_ode_contactgroup);
        dJointAttach(fixed_id, ODECaudier_L->m_ode_body, ODECaudier_R->m_ode_body);

        const dReal *rotation;
        rotation = dBodyGetRotation (ODECaudier_L->m_ode_body);

        cMatrix3d rrot_matt(rotation[0], rotation[1], rotation[2],
                           rotation[4], rotation[5], rotation[6],
                           rotation[8], rotation[9], rotation[10]);
        rrot_matt.rotateAboutLocalAxisDeg(cVector3d(0.0, 0.0, 1.0), 1.0);
        ODECaudier_L->setLocalRot(rrot_matt);


        const dReal *rotation1;
        rotation1 = dBodyGetRotation (ODECaudier_R->m_ode_body);

        cMatrix3d rrot_matt1(rotation1[0], rotation1[1], rotation1[2],
                             rotation1[4], rotation1[5], rotation1[6],
                             rotation1[8], rotation1[9], rotation1[10]);
        rrot_matt1.rotateAboutLocalAxisDeg(cVector3d(0.0, 0.0, 1.0), 1.0);
        ODECaudier_R->setLocalRot(rrot_matt1);

//        ODECaudier_L->rotateAboutLocalAxisDeg(cVector3d(0.0, 0.0, 1.0), 1.0);
    }

    else if (a_key == GLFW_KEY_N) {
        m_ODEBody0->setLocalPos(0.06, 0.1, 0.25);
    }

    else if (a_key == GLFW_KEY_B) {
        ODEWorld->setGravity(cVector3d(0.0, 0.0, -9.81));
    }
    else if (a_key == GLFW_KEY_V) {
        ODEWorld->setGravity(cVector3d(0.0, 0.0, 0.0));
    }


//    shaft->rotateExtrinsicEulerAnglesDeg(0.0, 90.0, 0.0, C_EULER_ORDER_XYZ);

}

//---------------------------------------------------------------------------
void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
                        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);

    // update information to scope
    scope_yaw->setSignalValues(desired_yaw, current_yaw);
    scope_pitch->setSignalValues(desired_pitch, current_pitch);
    scope_roll->setSignalValues(desired_roll, current_roll);


    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, false);

    // render world
    camera->renderView(width, height);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//---------------------------------------------------------------------------

void updateHaptics(void)
{

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // reset clock
    cPrecisionClock clock;
    clock.reset();

    // main haptic simulation loop
    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////
        // SIMULATION TIME
        /////////////////////////////////////////////////////////////////////

        // stop the simulation clock
        clock.stop();

        // read the time increment in seconds
        double timeInterval = cClamp(clock.getCurrentTimeSeconds(), 0.0001, 0.001);

        // restart the simulation clock
        clock.reset();
        clock.start();

        // update frequency counter
        freqCounterHaptics.signal(1);



        /////////////////////////////////////////////////////////////////////
        // HAPTIC FORCE COMPUTATION
        /////////////////////////////////////////////////////////////////////

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
//        tool->updateFromDevice();
//        gripper_tool->updateFromDevice();

        /////////////////////////////////////////////////////////////////////
        // DYNAMIC SIMULATION
        /////////////////////////////////////////////////////////////////////
        if (graspActive)
        {
            // check if button pressed
            if ((gripper_tool->getUserSwitch(0)) && graspObject != NULL)
            {
                // position grasp point in world coordinates
                cVector3d posA = graspObject->getLocalTransform() * graspPos;

                // position of tool
//                cVector3d posB = gripper_tool->getHapticPoint(0)->getGlobalPosGoal();
                cVector3d posB = gripper_tool->m_hapticPointThumb->getGlobalPosGoal();

                // update line
                graspLine->m_pointA = posA;
                graspLine->m_pointB = posB;
                graspLine->setShowEnabled(true);

                // compute force
                cVector3d force = 4.0 * (posB - posA);

                // apply force
                graspObject->addExternalForceAtPoint(force, posA);

                // apply reaction force to haptic device
                tool->addDeviceGlobalForce(-force);
                gripper_tool->addDeviceGlobalForce(-force);
            }
            else
            {
                graspLine->setShowEnabled(false);
                graspActive = false;
                graspObject = NULL;
            }
        }
        else
        {
            // get pointer to next interaction point of tool
            cHapticPoint* interactionPoint = tool->getHapticPoint(0);

            // check primary contact point if available
            if (interactionPoint->getNumCollisionEvents() > 0)
            {
                cCollisionEvent* collisionEvent = interactionPoint->getCollisionEvent(0);

                // given the mesh object we may be touching, we search for its owner which
                // could be the mesh itself or a multi-mesh object. Once the owner found, we
                // look for the parent that will point to the ODE object itself.
                cGenericObject* object = collisionEvent->m_object->getOwner()->getOwner();

                // cast to ODE object
                cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(object);

                // if ODE object, we apply interaction forces
                if (ODEobject != NULL)
                {
                    ODEobject->addExternalForceAtPoint(-5000.0 * interactionPoint->getLastComputedForce(), collisionEvent->m_globalPos);

                    // check if button pressed
                    if (tool->getUserSwitch(0))
                    {
                        graspObject = ODEobject;
                        graspPos = collisionEvent->m_localPos;
                        // graspActive = true;
                    }
                }
            }


            cHapticPoint* interactionPoint_Thumb = gripper_tool->m_hapticPointThumb;
            cHapticPoint* interactionPoint_Finger = gripper_tool->m_hapticPointFinger;

            // check primary contact point if available
            if (interactionPoint_Thumb->getNumCollisionEvents() > 0) {
                cCollisionEvent *collisionEvent_Thumb = interactionPoint_Thumb->getCollisionEvent(0);

                // given the mesh object we may be touching, we search for its owner which
                // could be the mesh itself or a multi-mesh object. Once the owner found, we
                // look for the parent that will point to the ODE object itself.
                cGenericObject* object_on_Thumb = collisionEvent_Thumb->m_object->getOwner()->getOwner();

                // cast to ODE object
                cODEGenericBody* ODEobject_Thumb = dynamic_cast<cODEGenericBody*>(object_on_Thumb);

                // if ODE object, we apply interaction forces
                if (ODEobject_Thumb != NULL)
                {
                    gripper_tool->setUserSwitch(0,true);

                    // check if button pressed
                    if (gripper_tool->getUserSwitch(0))
                    {
                        graspObject = ODEobject_Thumb;
                        graspPos = collisionEvent_Thumb->m_localPos;
                        graspActive = true;
//                        gripper_tool->setUserSwitch(0,true);
//                        cout << "##########" << gripper_tool->getUserSwitch(0) << endl;
                    }

                    ODEobject_Thumb->addExternalForceAtPoint(-0.4 * interactionPoint_Thumb->getLastComputedForce(), collisionEvent_Thumb->m_globalPos);

//                    // check if button pressed
//                    if (tool->getUserSwitch(0))
//                    {
//                        graspObject = ODEobject_Thumb;
//                        graspPos = collisionEvent_Thumb->m_localPos;
//                        graspActive = true;
//                        gripper_tool->setUserSwitch(0,true);
//                    }
                }

            }

            // check primary contact point if available
            if (interactionPoint_Finger->getNumCollisionEvents() > 0) {
                cCollisionEvent *collisionEvent_Finger = interactionPoint_Finger->getCollisionEvent(0);

                // given the mesh object we may be touching, we search for its owner which
                // could be the mesh itself or a multi-mesh object. Once the owner found, we
                // look for the parent that will point to the ODE object itself.
                cGenericObject* object_on_Finger = collisionEvent_Finger->m_object->getOwner()->getOwner();

                // cast to ODE object
                cODEGenericBody* ODEobject_Finger = dynamic_cast<cODEGenericBody*>(object_on_Finger);

                // if ODE object, we apply interaction forces
                if (ODEobject_Finger != NULL)
                {
                    ODEobject_Finger->addExternalForceAtPoint(-0.04 * interactionPoint_Finger->getLastComputedForce(), collisionEvent_Finger->m_globalPos);

                    // check if button pressed
                    if (gripper_tool->getUserSwitch(0))
                    {
                        graspObject = ODEobject_Finger;
                        graspPos = collisionEvent_Finger->m_localPos;
                        graspActive = true;
                    }
                }

            }

        }


//        ODECaudier_L->addExternalForce(cVector3d(-0.0, -0.0, 0.0));
//        ODECaudier_R->addExternalForce(cVector3d(0.0, 0.0, 0.0));

//        ODECaudier_L->rotateAboutLocalAxisDeg(cVector3d(0.0, 0.0, 1.0), -0.005);
//        ODECaudier_R->rotateAboutLocalAxisDeg(cVector3d(0.0, 0.0, 1.0), -0.005);


        // desired_yaw = -0.0002473;
        // desired_pitch = 0.000206407;
        // desired_roll = -0.705916;
        // initialised = true;


        fixed_id = dJointCreateFixed(ODEWorld->m_ode_world, ODEWorld->m_ode_contactgroup);
        dJointAttach(fixed_id, ODECaudier_L->m_ode_body, ODECaudier_R->m_ode_body);
        dJointSetFixed(fixed_id);

        ODECaudier_L->setShowFrame(true);
        // ODECaudier_R->setShowFrame(true);

        cVector3d ODECaudier_L_position_current = ODECaudier_L->getLocalPos();
        cVector3d ODECaudier_R_position_current = ODECaudier_R->getLocalPos();

        if (initialised) {

            double position_scale = 2.0;
            double force_scale = 1.0;
            cVector3d active_force_Caudier_L;
            cVector3d active_force_Caudier_R;

            // cout << position_scale * desired_positions.x() - ODECaudier_L_position_current.x() << endl;
            // cout << position_scale * desired_positions.y() - ODECaudier_L_position_current.y() << endl;
            // cout << position_scale * desired_positions.z() - ODECaudier_L_position_current.z() << endl;

            if ( ((position_scale * desired_positions.x() - ODECaudier_L_position_current.x()) >= 0.001) ||
                 ((position_scale * desired_positions.y() - ODECaudier_L_position_current.y()) >= 0.001) ||
                 ((position_scale * desired_positions.z() - ODECaudier_L_position_current.z()) >= 0.001) ||
                 ((position_scale * desired_positions.x() - ODECaudier_L_position_current.x()) <= -0.001) ||
                 ((position_scale * desired_positions.y() - ODECaudier_L_position_current.y()) <= -0.001) ||
                 ((position_scale * desired_positions.z() - ODECaudier_L_position_current.z()) <= -0.001)
               ) 
            {
                active_force_Caudier_L = cVector3d(position_scale * desired_positions.x() - ODECaudier_L_position_current.x(),
                                                             position_scale * desired_positions.y() - ODECaudier_L_position_current.y(),
                                                             position_scale * desired_positions.z() - ODECaudier_L_position_current.z() + 9.81*(ODECaudier_L->getMass())/force_scale);
                active_force_Caudier_R = cVector3d(position_scale * desired_positions.x() - ODECaudier_R_position_current.x(),
                                                             position_scale * desired_positions.y() - ODECaudier_R_position_current.y(),
                                                             position_scale * desired_positions.z() - ODECaudier_R_position_current.z() + 9.81*(ODECaudier_R->getMass())/force_scale);
                // cout << active_force_Caudier_L << endl;
            }
            else {
                active_force_Caudier_L = cVector3d(0.0, 0.0, 9.81*(ODECaudier_L->getMass())/force_scale);
                active_force_Caudier_R = cVector3d(0.0, 0.0, 9.81*(ODECaudier_L->getMass())/force_scale);
            }

            // cout << active_force_Caudier_L << endl;

            ODECaudier_L->addExternalForce(force_scale*active_force_Caudier_L);
            ODECaudier_R->addExternalForce(force_scale*active_force_Caudier_R);


            cMatrix3d L_global_rot_mat_current = ODECaudier_L->getGlobalRot();


            tf2::Matrix3x3 tf_L_global_rot_mat_current;
            tf_L_global_rot_mat_current = tf2::Matrix3x3(
                L_global_rot_mat_current.getRow(0).x(), L_global_rot_mat_current.getRow(0).y(), L_global_rot_mat_current.getRow(0).z(), 
                L_global_rot_mat_current.getRow(1).x(), L_global_rot_mat_current.getRow(1).y(), L_global_rot_mat_current.getRow(1).z(),
                L_global_rot_mat_current.getRow(2).x(), L_global_rot_mat_current.getRow(2).y(), L_global_rot_mat_current.getRow(2).z());

            tf_L_global_rot_mat_current.getEulerYPR(current_yaw, current_roll, current_pitch, 1);

            double active_global_torque_yaw_Caudier_L = 0.0;
            double active_global_torque_pitch_Caudier_L = 0.0;
            double active_global_torque_roll_Caudier_L = 0.0;

            double yaw_Error = 0.0;
            double pitch_Error = 0.0;
            double roll_Error = 0.0;

            yaw_Error = desired_yaw - current_yaw;
            pitch_Error = desired_pitch - current_pitch;
            roll_Error = desired_roll - current_roll;

            double torque_yaw_Kp = 0.02;
            double torque_yaw_Ki = 0.000001;
            double torque_yaw_Kd = 100.0;

            double torque_pitch_Kp = 0.02;
            double torque_pitch_Ki = 0.0000001;
            double torque_pitch_Kd = 100.0;

            double torque_roll_Kp = 0.02;
            double torque_roll_Ki = 0.0000001;
            double torque_roll_Kd = 100.0;

            integral_yaw = integral_yaw + yaw_Error;
            integral_pitch = integral_pitch + pitch_Error;
            integral_roll = integral_roll + roll_Error;

            active_global_torque_yaw_Caudier_L = torque_yaw_Kp * yaw_Error +
                                                    torque_yaw_Ki * integral_yaw + 
                                                    torque_yaw_Kd * (yaw_Error - previous_yaw_Error);
            active_global_torque_pitch_Caudier_L = torque_pitch_Kp * pitch_Error +
                                                    torque_pitch_Ki * integral_pitch + 
                                                    torque_pitch_Kd * (pitch_Error - previous_pitch_Error);
            active_global_torque_roll_Caudier_L = torque_roll_Kp * roll_Error +
                                                    torque_roll_Ki * integral_roll + 
                                                    torque_roll_Kd * (roll_Error - previous_roll_Error);

            // cout << "desired Yaw : " << desired_yaw << "  current Yaw : " << current_yaw << endl;
            // cout << "Yaw Torque : " << active_global_torque_yaw_Caudier_L << endl;
            // cout << "Yaw Error : " << yaw_Error << endl;
            // cout << "Integral Yaw Error : " << integral_yaw << endl << "---------------------------------------------------" << endl;
            // cout << "desired Pitch : " << desired_pitch << "  current Pitch : " << current_pitch << endl;
            // cout << "Pitch Torque : " << active_global_torque_pitch_Caudier_L << endl;
            // cout << "Pitch Error : " << pitch_Error << endl;
            // cout << "Integral Pitch Error : " << integral_pitch << endl << "---------------------------------------------------" << endl;
            // cout << "desired Roll : " << desired_roll << "  current Roll : " << current_roll << endl;
            // cout << "Roll Torque : " << active_global_torque_roll_Caudier_L << endl;
            // cout << "Roll Error : " << roll_Error << endl;
            // cout << "Integral Roll Error : " << integral_roll << endl << "####################################################" << endl;


            dBodyAddTorque(ODECaudier_L->m_ode_body, active_global_torque_pitch_Caudier_L, active_global_torque_roll_Caudier_L, active_global_torque_yaw_Caudier_L);
            dBodyAddTorque(ODECaudier_L->m_ode_body, active_global_torque_pitch_Caudier_L, active_global_torque_roll_Caudier_L, active_global_torque_yaw_Caudier_L);

            // TRY REV TORQUE !!!!!!!!!!!!!!!!!!!!!!!!!!!

            previous_yaw_Error = yaw_Error;
            previous_pitch_Error = pitch_Error;
            previous_roll_Error = roll_Error;


            // dBodySetQuaternion(ODECaudier_L->m_ode_body, desired_quat);
            // dBodySetQuaternion(ODECaudier_R->m_ode_body, desired_quat);
            // ODECaudier_L->rotateAboutGlobalAxisDeg(cVector3d(0.0, 1.0, 0.0), 0.0);
            // ODECaudier_R->rotateAboutGlobalAxisDeg(cVector3d(0.0, 1.0, 0.0), 0.0);

            // ODECaudier_L->setLocalRot(rotation_mat1);

            // ODECaudier_R->setLocalRot(rotation_mat2);

        }

        // dBodyAddRelTorque(ODECaudier_L->m_ode_body, 0.0, 0.0, 0.002);
        // dBodyAddRelTorque(ODECaudier_R->m_ode_body, 0.0, 0.0, 0.002);

        
        



//        const dReal *rotation;
//        rotation = dBodyGetRotation (ODECaudier_L->m_ode_body);
//
//        cMatrix3d rrot_matt(rotation[0], rotation[1], rotation[2],
//                            rotation[4], rotation[5], rotation[6],
//                            rotation[8], rotation[9], rotation[10]);
//        rrot_matt.rotateAboutLocalAxisDeg(cVector3d(0.0, 0.0, 1.0), -0.001);
//        ODECaudier_L->setLocalRot(rrot_matt);


//        const dReal *rotation1;
//        rotation1 = dBodyGetRotation (ODECaudier_R->m_ode_body);
//
//        cMatrix3d rrot_matt1(rotation1[0], rotation1[1], rotation1[2],
//                             rotation1[4], rotation1[5], rotation1[6],
//                             rotation1[8], rotation1[9], rotation1[10]);
//        rrot_matt1.rotateAboutLocalAxisDeg(cVector3d(0.0, 0.0, 1.0), -0.001);
//        ODECaudier_R->setLocalRot(rrot_matt1);


    //    ODECaudier_L->setLocalRot(rotation_mat1);

    //    ODECaudier_R->setLocalRot(rotation_mat2);

//        ODECaudier_L->rotateAboutLocalAxisDeg(cVector3d(0.0, 0.0, 1.0), -0.005);
//        ODECaudier_R->rotateAboutLocalAxisDeg(cVector3d(0.0, 0.0, 1.0), -0.005);



        // get rotation
//        const dReal *rotation;
//        rotation = dBodyGetRotation (ODECaudier_L->m_ode_body);
//
//        cout << rotation[0] << "   " << rotation[1] << "   " << rotation[2] << endl;
//        cout << rotation[4] << "   " << rotation[5] << "   " << rotation[6] << endl;
//        cout << rotation[8] << "   " << rotation[9] << "   " << rotation[10] << endl;


//        ODECaudier_shaft->addExternalForce(cVector3d(0.0, 0.0, 0.0));

//        dJointSetFeedback(hinge_id, hinge_id->feedback);
//        cout << dAreConnectedExcluding(ODECaudier_L->m_ode_body, ODECaudier_shaft->m_ode_body, 5) << endl;
//        cout << dJointGetNumBodies(hinge_id) << endl;


//        hinge_id = dJointCreateHinge(ODEWorld->m_ode_world, ODEWorld->m_ode_contactgroup);
//        dJointAttach(hinge_id, ODECaudier_L->m_ode_body, ODECaudier_R->m_ode_body);



//        hinge_id2 = dJointCreateHinge(ODEWorld->m_ode_world, ODEWorld->m_ode_contactgroup);
//        dJointAttach(hinge_id2, ODECaudier_shaft->m_ode_body, ODECaudier_R->m_ode_body);
//        cout << dJointGetNumBodies(hinge_id2) << endl;

//        dJointAddHingeTorque(hinge_id, 0.001);

//        cVector3d ODECaudier_L_pos = ODECaudier_L->getLocalPos();
//        cMatrix3d the_rot_L(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
//        the_rot_L.rotateAboutGlobalAxisDeg(cVector3d(0.0, 1.0, 0.0), 10.0);
//        ODECaudier_L->setLocalTransform(cTransform(cVector3d(ODECaudier_pos.x()+0.00001, 0.0, ODECaudier_pos.z()), the_rot_L));
//        ODECaudier_L->setLocalTransform(cTransform(cVector3d(ODECaudier_L_pos.x()+0.000001, ODECaudier_L_pos.y(), ODECaudier_L_pos.z()), the_rot_L));

//        cVector3d ODECaudier_shaft_pos = ODECaudier_shaft->getLocalPos();
//        cMatrix3d the_rot_shaft(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
//        the_rot_L.rotateAboutGlobalAxisDeg(cVector3d(0.0, 1.0, 0.0), 10.0);
//        ODECaudier_L->setLocalTransform(cTransform(cVector3d(ODECaudier_pos.x()+0.00001, 0.0, ODECaudier_pos.z()), the_rot_L));
//        ODECaudier_shaft->setLocalTransform(cTransform(cVector3d(ODECaudier_shaft_pos.x()+0.000001, ODECaudier_shaft_pos.y(), ODECaudier_shaft_pos.z()), the_rot_shaft));

//        cVector3d ODECaudier_R_pos = ODECaudier_R->getLocalPos();
//        cMatrix3d the_rot_R(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
//        the_rot_R.rotateAboutGlobalAxisDeg(cVector3d(0.0, 1.0, 0.0), 180.0);
//        ODECaudier_L->setLocalTransform(cTransform(cVector3d(ODECaudier_pos.x()+0.00001, 0.0, ODECaudier_pos.z()), the_rot_L));
//        ODECaudier_R->setLocalTransform(cTransform(cVector3d(ODECaudier_R_pos.x()+0.000001, ODECaudier_R_pos.y(), ODECaudier_R_pos.z()), the_rot_R));

//        dJointSetHingeAnchor(hinge_id, ODECaudier_shaft_pos.x(), ODECaudier_shaft_pos.y(), ODECaudier_shaft_pos.z());

//        dVector3 result;
//        dJointGetHingeAnchor(hinge_id,result);
//        cout << result[0] << "  " << result[1] << "  " << result[2] << endl;




//        dJointSetHingeAxis(hinge_id, 1, 0, 0);
//
//        dVector3 result2;
//        dJointGetHingeAxis(hinge_id,result2);
//        cout << result2[0] << "  " << result2[1] << "  " << result2[2] << endl;
//
//        cout << dJointGetHingeAngle(hinge_id) << endl;
//        dJointAddHingeTorque(hinge_id, -0.0001);


        //        cout << "graspActive : " << graspActive << endl;



//        // changing gripper angle and log
//        gripper_tool->getHapticDevice()->m_specifications.m_gripperMaxAngleRad = cDegToRad(gripper_angle_lim_max);
//        if (gripper_angle < gripper_angle_lim_max){
//            gripper_angle = gripper_angle + 0.001;
//            gripper_tool->setDeviceLocalPos(cVector3d(0.0,0.0,0.0));
//            gripper_tool->setLocalPos(0.1+gripper_angle/150, 0.0, 40.0/1000 - gripper_angle/1000);
//        }
//        else {
//            gripper_angle = 0.0;
//        }
        gripper_tool->setGripperAngleDeg(gripper_angle);

//        gripper_tool->setDeviceLocalPos(cVector3d(0.0,0.0,0.0));
//        gripper_tool->setLocalPos(0.1+gripper_angle/150, 0.0, 40.0/1000 - gripper_angle/1000);
//        gripper_tool->setGripperAngleDeg(15.0);

//        cout << "current gripper angle is : " << gripper_tool->getGripperAngleDeg() << endl;

        // changing the gripper device Global pos
//        gripper_tool->setDeviceGlobalPos(cVector3d(0.1,0.0,0.0));

        // change the
//        gripper_tool->setDeviceLocalPos(cVector3d(0.0,0.0,0.0));
//        gripper_tool->setLocalPos(0.1, 0.0, 0.0);

        cMatrix3d test_rot_mat2(1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0);
        test_rot_mat2.rotateAboutGlobalAxisDeg(cVector3d(0.0, 1.0, 0.0), 90.0);
        gripper_tool->setLocalRot(test_rot_mat2);

//        cMatrix3d test_rot_mat(1.0, 0.0, 0.0,
//                               0.0, 1.0, 0.0,
//                               0.0, 0.0, 1.0);
//        test_rot_mat.rotateAboutGlobalAxisDeg(cVector3d(0.0, 1.0, 0.0), 0.0);
//        gripper_tool->setDeviceLocalRot(test_rot_mat);


//        shaft->rotateExtrinsicEulerAnglesDeg(0.0, 90.0, 0.0, C_EULER_ORDER_XYZ);

//        cout << "right hand ? " << gripper_tool->getHapticDevice()->m_specifications.m_rightHand << endl;
//        cout << "left hand ? " << gripper_tool->getHapticDevice()->m_specifications.m_leftHand << endl;

//        cout << "shaft orientation is : " << endl
//                                          << shaft->getLocalRot().getRow(0) << endl
//                                          << shaft->getLocalRot().getRow(1) << endl
//                                          << shaft->getLocalRot().getRow(2) << endl;
//
//        cout << "device global pos is : " << gripper_tool->getDeviceGlobalPos() << endl;

        // cVector3d body2_pos = m_ODEBody2->getLocalPos();
//        cout << body2_pos.z() << endl;
//        m_ODEBody2->setLocalPos(body2_pos.x()+0.00001 , body2_pos.y(), body2_pos.z());

        // cVector3d body1_pos = m_ODEBody1->getLocalPos();
//        cout << body1_pos.z() << endl;
//        m_ODEBody1->setLocalPos(body1_pos.x()-0.00001 , body1_pos.y(), body1_pos.z());

//        m_ODEBody2->setLocalRot(cMatrix3d(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0));
        // cMatrix3d the_rot(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        // the_rot.rotateAboutGlobalAxisDeg(cVector3d(0.0, 1.0, 0.0), 10.0);
        // m_ODEBody1->setLocalTransform(cTransform(cVector3d(0.14, 0.0, body1_pos.z()+0.0004), the_rot));
        // the_rot.rotateAboutGlobalAxisDeg(cVector3d(0.0, 1.0, 0.0), -20.0);
        // m_ODEBody2->setLocalTransform(cTransform(cVector3d(0.11, 0.0, body2_pos.z()+0.0004), the_rot));

        // update simulation
        ODEWorld->updateDynamics(timeInterval);

        // compute interaction forces
//        tool->computeInteractionForces();
        gripper_tool->computeInteractionForces();



        // send forces to haptic device
//        tool->applyToDevice();
//        gripper_tool->applyToDevice();
    }

    // exit haptics thread
    simulationFinished = true;
}

//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    tool->stop();

    // delete resources
    delete hapticsThread;
    delete world;
    delete handler;
}





